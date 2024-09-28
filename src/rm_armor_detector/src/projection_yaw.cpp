#include "projection_yaw.hpp"

ProjectionYaw::ProjectionYaw()
{
    std::cout << "ProjectionYaw constructed!" << std::endl;
    parameters_init();
}

void ProjectionYaw::parameters_init()
{
    big_armor_world_points_ = {
        cv::Point3f(-0.115, 0.0265, 0.),
        cv::Point3f(-0.115, -0.0265, 0.),
        cv::Point3f(0.115, -0.0265, 0.),
        cv::Point3f(0.115, 0.0265, 0.)
    };

    small_armor_world_points_ = {
        cv::Point3f(-0.068, 0.0275, 0.),
        cv::Point3f(-0.068, -0.0275, 0.),
        cv::Point3f(0.068, -0.0275, 0.),
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
}

double ProjectionYaw::get_cost(const std::vector<cv::Point2f>& pred_points, 
                                const std::vector<cv::Point2f>& points,const double& pred_yaw)
{
    std::size_t size = pred_points.size();
    double cost = 0.0;
    for(std::size_t i = 0; i < size; i++)
    {
        std::size_t p = (i + 1u) % size;
        cv::Point2f pred_point_standard = pred_points[p] - pred_points[i];
        cv::Point2f point_standard = points[p] - points[i];
        double pixel_dis = (0.5 * ((pred_points[i] - points[i]).norm() + (pred_points[p] - points[p]).norm())
                            + std::fabs(pred_point_standard.norm() - point_standard.norm())) / pred_point_standard.norm();
        double angular_dis = pred_point_standard.norm() * get_abs_angle(pred_point_standard, point_standard) / pred_point_standard.norm();
        double cost_i = std::pow(pixel_dis * std::sin(pred_yaw), 2) + std::pow(angular_dis * std::cos(pred_yaw), 2);
        cost += std::sqrt(cost_i);
    }
    return cost;
}

double ProjectionYaw::get_abs_angle(const cv::Point2f& first_point, const cv::Point2f& second_point)
{
    if(first_point.norm() == 0 || second_point.norm() == 0)
    {
        std::cout << "Error: The norm of the point is zero!" << std::endl;
        return 0;
    }
    double cos_theta = (first_point.x * second_point.x + first_point.y * second_point.y) / (first_point.norm() * second_point.norm());
    return std::acos(cos_theta);
}

// 利用三分法计算极小值，不断更新预测的偏航角
double ProjectionYaw::update_pred_yaw(double left_yaw, double right_yaw,const int& ITERATIONS_NUM)
{
    double phi = (std::sqrt(5.0) - 1.0) / 2.0;
    int choice = -1;
    double left_cost = 0.0;
    double right_cost = 0.0;
    for(int i = 0; i < ITERATIONS_NUM; i++)
    {
        double ml = left_yaw + (right_yaw - left_yaw) * (1 - phi);
        double mr = left_yaw + (right_yaw - left_yaw) * phi;
        if(choice != 0)
        {
            left_cost = get_cost(ml);
        }
        if(choice != 1)
        {
            right_cost = get_cost(mr);
        }
        if(left_cost < right_cost)
        {
            right_yaw = mr;
            right_cost = left_cost;
            choice = 1;
        }
        else
        {
            left_yaw = ml;
            left_cost = right_cost;
            choice = 0;
        }
    }
    return (left_yaw + right_yaw) / 2;
}

std::vector<cv::Point3f> ProjectionYaw::get_armor_points(
    const DecisionArmor& decision_armor,
    const double& pitch,
    const double& pred_yaw
){
    std::vector<cv::Point3f> armor_world_points;
    if(decision_armor.is_big_armor == true)
    {
        armor_world_points = big_armor_world_points_;
    }
    else if(decision_armor.is_big_armor == false)
    {
        armor_world_points = small_armor_world_points_;
    }

    // 将装甲板坐标变换到horizontal平面
    geometry_msgs::msg::PointStamped armor_center;
    armor_center.point.x = decision_armor.pose.position.x;
    armor_center.point.y = decision_armor.pose.position.y;
    armor_center.point.z = decision_armor.pose.position.z;
    armor_center.header = decision_armor.pose.header;
    tf2_buffer_->transform(armor_center, armor_center, "horizontal_camera_link", rclcpp::Duration(0.1));

    cv::Mat radius_vec = (cv::Mat_<double>(2, 1) << 0, 1);
    radius_vec = rotate(radius_vec, pred_yaw);
    cv::Mat x_2_vec = rotate(radius_vec, CV_PI / 2);
    cv::Mat x_vec = (cv::Mat_<double>(3, 1) << x_2_vec.at<double>(0), x_2_vec.at<double>(1), 0);
    
}

// 将二维向量逆时针旋转角度angle
// vec : 2 * 1的矩阵
// angle : 旋转角度 弧度制
cv::Mat ProjectionYaw::rotate(const cv::Mat& vec, const double& angle)
{
    cv::Mat rotate_mat = (cv::Mat_<double>(2, 2) << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle));
    return rotate_mat * vec;
}