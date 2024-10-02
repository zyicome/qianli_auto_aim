#include "closed_loop_node.hpp"

namespace rm_closed_loop
{

ClosedLoopNode::ClosedLoopNode(const rclcpp::NodeOptions & options) : Node("closed_loop_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting ClosedLoopNode!");
    parameters_init();
    //test();
}

void ClosedLoopNode::parameters_init()
{
    RCLCPP_INFO(this->get_logger(), "Begin to init ClosedLoopNode parameters!");

    PITCH_ = 15.0 * CV_PI / 180.0;

    // 1  2
    // 4  3
    big_armor_world_points_ = {
        cv::Point3f(0.115, -0.0265, 0.),
        cv::Point3f(-0.115, -0.0265, 0.),
        cv::Point3f(-0.115, 0.0265, 0.),
        cv::Point3f(0.115, 0.0265, 0.)
    };

    small_armor_world_points_ = {
        cv::Point3f(0.068, -0.0275, 0.),
        cv::Point3f(-0.068, -0.0275, 0.),
        cv::Point3f(-0.068, 0.0275, 0.),
        cv::Point3f(0.068, 0.0275, 0.)
    };

    // Subscriber with tf2 message_filter
    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Sub
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&ClosedLoopNode::camera_info_callback, this, std::placeholders::_1));

    trajectory_closed_loop_sub_ = this->create_subscription<rm_msgs::msg::ClosedLoop>(
        "trajectory/closed_loop", 10, std::bind(&ClosedLoopNode::trajectory_closed_loop_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Finished init ClosedLoopNode parameters successfully!");
}

void ClosedLoopNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Closed_loop_node received camera info!");
    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
    distortion_coefficients_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();

    std::cout << "camera_matrix: " << camera_matrix_ << std::endl;
    std::cout << "distortion_coefficients: " << distortion_coefficients_ << std::endl;
}

void ClosedLoopNode::trajectory_closed_loop_callback(const rm_msgs::msg::ClosedLoop::SharedPtr msg)
{
    double yaw = msg->yaw;
    tf2::Quaternion tf2_q;
    tf2_q.setRPY( - 75.0 * CV_PI / 180.0, yaw * CV_PI / 180.0, 0.0 ); // 有问题？对于不同的旋转顺序，结果不同？如何解决？
    msg->now_pose.orientation = tf2::toMsg(tf2_q);
    try
    {
        geometry_msgs::msg::TransformStamped transformStamped = tf2_buffer_->lookupTransform(
            "odom", "camera_optical_frame", 
            msg->image_header.stamp);

        tf2::doTransform(msg->now_pose, msg->now_pose, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }
}

void ClosedLoopNode::draw_armor_on_image(cv::Mat image, const geometry_msgs::msg::PoseStamped & armor_pose, std::string id, int armor_num, double r, double another_r, double dz, double yaw)
{
    cv::Point3d car_center = cv::Point3d(armor_pose.pose.position.x, armor_pose.pose.position.y, armor_pose.pose.position.z);
    std::vector<cv::Point3d> armor_3d_points;
    std::vector<cv::Point2d> armor_image_points;
    std::vector<std::vector<cv::Point2d>> all_armor_image_points;
    if(armor_num == 2)
    {
        for(size_t i = 0; i < 2; i++)
        {
            double x = car_center.x + r * std::cos(i * CV_PI + yaw);
            double y = car_center.y + r * std::sin(i * CV_PI + yaw);
            cv::Point3d armor_center = cv::Point3d(x, y, car_center.z); 
            armor_3d_points = get_armor_3d_points(armor_center, big_armor_world_points_, yaw);
            armor_image_points = get_armor_image_points(armor_3d_points);
            all_armor_image_points.push_back(armor_image_points);
        }
    }
    else if(armor_num == 3)
    {
        for(size_t i = 0; i < 3; i++)
        {
            double x = car_center.x + r * std::cos(i * CV_PI * 2 / 3 + yaw);
            double y = car_center.y + r * std::sin(i * CV_PI * 2 / 3 + yaw);
            cv::Point3d armor_center = cv::Point3d(x, y, car_center.z); 
            armor_3d_points = get_armor_3d_points(armor_center, small_armor_world_points_, yaw);
            armor_image_points = get_armor_image_points(armor_3d_points);
            all_armor_image_points.push_back(armor_image_points);
        }
    }
    else if(armor_num == 4 && id != "1")
    {
        for(size_t i = 0; i < 4; i++)
        {
            double x = 0.0;
            double y = 0.0;
            cv::Point3d armor_center;
            if(i == 0 || i == 2)
            {
                x = car_center.x + r * std::cos(i * CV_PI / 2 + yaw);
                y = car_center.y + r * std::sin(i * CV_PI / 2 + yaw);
                armor_center = cv::Point3d(x, y, car_center.z); 
            }
            else if(i == 1 || i == 3)
            {
                x = car_center.x + another_r * std::cos(i * CV_PI / 2 + yaw);
                y = car_center.y + another_r * std::sin(i * CV_PI / 2 + yaw);
                armor_center = cv::Point3d(x, y, car_center.z + dz); 
            }
            armor_3d_points = get_armor_3d_points(armor_center, small_armor_world_points_, yaw);
            armor_image_points = get_armor_image_points(armor_3d_points);
            all_armor_image_points.push_back(armor_image_points);
        }
    }
    else if(armor_num == 4 && id == "1")
    {
        for(size_t i = 0; i < 4; i++)
        {
            double x = 0.0;
            double y = 0.0;
            cv::Point3d armor_center;
            if(i == 0 || i == 2)
            {
                x = car_center.x + r * std::cos(i * CV_PI / 2 + yaw);
                y = car_center.y + r * std::sin(i * CV_PI / 2 + yaw);
                armor_center = cv::Point3d(x, y, car_center.z); 
            }
            else if(i == 1 || i == 3)
            {
                x = car_center.x + another_r * std::cos(i * CV_PI / 2 + yaw);
                y = car_center.y + another_r * std::sin(i * CV_PI / 2 + yaw);
                armor_center = cv::Point3d(x, y, car_center.z + dz); 
            }
            armor_3d_points = get_armor_3d_points(armor_center, big_armor_world_points_, yaw);
            armor_image_points = get_armor_image_points(armor_3d_points);
            all_armor_image_points.push_back(armor_image_points);
        }
    }
    cv::Point2d car_center_image_point = get_armor_image_points({car_center})[0];

    // 将一个装甲板的四个点连接起来
    for(size_t i = 0; i < all_armor_image_points.size(); i++)
    {
        for(size_t j = 0; j < all_armor_image_points[i].size(); j++)
        {
            cv::line(image, all_armor_image_points[i][j], all_armor_image_points[i][(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
    // 标出车中心
    cv::circle(image, car_center_image_point, 5, cv::Scalar(0, 0, 255), -1);
    // 将车中心与装甲板中心连接起来
    for(size_t i = 0; i < all_armor_image_points.size(); i++)
    {
        double armor_center_image_x = (all_armor_image_points[i][0].x + all_armor_image_points[i][2].x) / 2;
        double armor_center_image_y = (all_armor_image_points[i][0].y + all_armor_image_points[i][2].y) / 2;
        cv::Point2d armor_center_image_point = cv::Point2d(armor_center_image_x, armor_center_image_y);
        cv::line(image, car_center_image_point, armor_center_image_point, cv::Scalar(0, 255, 0), 2);
    }
    // 将前一个装甲板的两个点与后一个装甲板的两个点连接起来
    for(size_t i = 0; i < all_armor_image_points.size() ; i++)
    {
        for(size_t j = (i + 1) % all_armor_image_points.size() ; j < all_armor_image_points.size(); j++)
        {
            cv::line(image, all_armor_image_points[i][1], all_armor_image_points[j][0], cv::Scalar(0, 255, 0), 2);
            cv::line(image, all_armor_image_points[i][2], all_armor_image_points[j][3], cv::Scalar(0, 255, 0), 2);
        }
    }
}

std::vector<cv::Point3d> ClosedLoopNode::get_armor_3d_points(cv::Point3d & armor_center, std::vector<cv::Point3d> armor_world_points, double yaw)
{
    cv::Mat radius_vec = (cv::Mat_<double>(2, 1) << 0, 1);
    radius_vec = rotate(radius_vec, yaw);
    cv::Mat xz_2_vec = rotate(radius_vec, - CV_PI / 2);
    cv::Mat xz_vec = (cv::Mat_<double>(3, 1) << xz_2_vec.at<double>(0), xz_2_vec.at<double>(1), 0);
    cv::Mat y_vec = (cv::Mat_<double>(3, 1) << radius_vec.at<double>(0) * std::sin(PITCH_), radius_vec.at<double>(1) * std::sin(PITCH_)
                                                ,  std::cos(PITCH_));
    std::vector<cv::Point3d> armor_points;
    for(size_t i = 0; i < armor_world_points.size(); i++)
    {
        cv::Mat xz_trans = armor_world_points[i].x * xz_vec;
        // xzy 转为 xyz
        cv::Point3d xz_trans_point = cv::Point3f(xz_trans.at<double>(0,0),xz_trans.at<double>(2,0),xz_trans.at<double>(1,0));
        cv::Mat y_trans = armor_world_points[i].y * y_vec;
        // xzy 转为 xyz
        cv::Point3d y_trans_point = cv::Point3f(y_trans.at<double>(0,0),y_trans.at<double>(2,0),y_trans.at<double>(1,0));
        armor_points.push_back(armor_center + xz_trans_point + y_trans_point);
    }
    return armor_points;
}

std::vector<cv::Point2d> ClosedLoopNode::get_armor_image_points(const std::vector<cv::Point3d> & armor_3d_points)
{
    std::vector<cv::Point2d> armor_image_points;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(armor_3d_points, rvec, tvec, camera_matrix_, distortion_coefficients_, armor_image_points);
    return armor_image_points;
}

// 将二维向量逆时针旋转角度angle
// vec : 2 * 1的矩阵
// angle : 旋转角度 弧度制
cv::Mat ClosedLoopNode::rotate(const cv::Mat& vec, const double& angle)
{
    cv::Mat rotate_mat = (cv::Mat_<double>(2, 2) << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle));
    return rotate_mat * vec;
}

} // namespace rm_closed_loop

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_closed_loop::ClosedLoopNode)