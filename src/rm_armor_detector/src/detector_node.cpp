#include "detector_node.hpp"

namespace rm_armor_detector
{

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options) : Node("armor_detector_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting ArmorDetectorNode!");
    robots_init();
    parameters_init();
    //test();
}

void ArmorDetectorNode::parameters_init()
{
    RCLCPP_INFO(this->get_logger(), "Begin to init parameters!");
    this->declare_parameter("detector_debug", 0);
    is_debug_ = this->get_parameter("detector_debug").as_int();
    std::cout << "is_detector_debug: " << is_debug_ << std::endl;
    if(is_debug_ == true)
    {
        create_debug_publishers();
    }
    // Debug param change moniter
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("detector_debug", [this](const rclcpp::Parameter & p) {
      is_debug_ = p.as_bool();
      is_debug_ ? create_debug_publishers() : destroy_debug_publishers();
    });

    this->declare_parameter("MIN_BIG_ARMOR_RATIO", 3.2);
    MIN_BIG_ARMOR_RATIO_ = this->get_parameter("MIN_BIG_ARMOR_RATIO").as_double();
    std::cout << "MIN_BIG_ARMOR_RATIO: " << MIN_BIG_ARMOR_RATIO_ << std::endl;

    this->declare_parameter("detect_color", 2);
    detect_color_ = this->get_parameter("detect_color").as_int();
    std::cout << "detect_color: " << detect_color_ << std::endl;

    this->declare_parameter("priority_armors", std::vector<int>());
    auto priority_armors_long = this->get_parameter("priority_armors").as_integer_array();
    std::vector<int> priority_armors;
    std::transform(priority_armors_long.begin(), priority_armors_long.end(), std::back_inserter(priority_armors),
                    [](long int val) { return static_cast<int>(val); });
    if(priority_armors.size() != decision_armors_.size())
    {
        RCLCPP_ERROR(this->get_logger(), "priority_armors size is != decision_armors_ size !");
        return;
    }
    for(size_t i = 0;i<decision_armors_.size();i++)
    {
        decision_armors_[i].priority = priority_armors[i];
    }
    for(size_t i = 0;i<decision_armors_.size();i++)
    {
        std::cout << "priority_armors: " << decision_armors_[i].priority << std::endl;
    }

    this->declare_parameter("ignore_armors", std::vector<int>());
    auto ignore_armors_long = this->get_parameter("ignore_armors").as_integer_array();
    ignore_armors_.clear();
    std::transform(ignore_armors_long.begin(), ignore_armors_long.end(), std::back_inserter(ignore_armors_),
                    [](long int val) { return static_cast<int>(val); });
    for(size_t i = 0;i<ignore_armors_.size();i++)
    {
        std::cout << "ignore_armors: " << ignore_armors_[i] << std::endl;
    }

    std::string model_path = "/home/zyicome/zyb/qianli_auto_aim/src/rm_armor_detector/model/four_points_armor/armor.onnx";
    openvino_detector_ = std::make_shared<OpenvinoDetector>();
    openvino_detector_->set_onnx_model(model_path, "CPU");

    armor_pub_ = this->create_publisher<rm_msgs::msg::Armor>("/detector/armor", 10);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::camera_info_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Finished init parameters successfully!");
}

void ArmorDetectorNode::create_debug_publishers()
{
    result_image_pub_ = image_transport::create_publisher(this, "/detector/result_image");
}

void ArmorDetectorNode::destroy_debug_publishers()
{
    result_image_pub_.shutdown();
}

void ArmorDetectorNode::debug_deal(const cv::Mat &image, const std::vector<Armor> &armors, const DecisionArmor &decision_armor)
{
    if(is_debug_ == true)
    {
        cv::Mat debug_image = image.clone();
        // 绘制识别到的装甲板框框
        for(size_t i =0;i<armors.size();i++)
        {
            // Convert vector<cv::Point2f> to vector<vector<cv::Point>>
            std::vector<std::vector<cv::Point>> pts(1);
            for (const auto& p : armors[i].four_points) {
                pts[0].emplace_back(cv::Point(p.x, p.y));
            }
            if(armors[i].id == decision_armor.id)
            {
                cv::polylines(debug_image, pts, true, cv::Scalar(0, 0, 255), 2);
                //绘制距离
                cv::putText(debug_image, std::to_string(decision_armor.distance), cv::Point((armors[i].four_points[0].x + armors[i].four_points[2].x) / 2, (armors[i].four_points[0].y + armors[i].four_points[2].y) / 2), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 5);
            }
            else
            {
                cv::polylines(debug_image, pts, true, cv::Scalar(0, 255, 0), 2);
            }
            cv::putText(debug_image, armors[i].name, cv::Point(armors[i].rect.x, armors[i].rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 5);
        }
        // 绘制图片大小以及图片中心
        cv::putText(debug_image, "width: " + std::to_string(debug_image.cols) + " height: " + std::to_string(debug_image.rows), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 5);
        cv::circle(debug_image, image_center_, 5, cv::Scalar(0, 255, 0), -1);
        // 绘制fps
        cv::putText(debug_image, "detector fps: " + std::to_string(detector_now_fps), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 5);
        sensor_msgs::msg::Image debug_image_msg = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg());
        result_image_pub_.publish(debug_image_msg);
    }
}

void ArmorDetectorNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Begin to receive camera info!");
    camera_width_ = msg->width;
    camera_height_ = msg->height;
    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
    distortion_coefficients_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();
    image_center_ = cv::Point2f(camera_width_ / 2, camera_height_ / 2);
    
    std::cout << "camera_width: " << camera_width_ << std::endl;
    std::cout << "camera_height: " << camera_height_ << std::endl;
    std::cout << "camera_matrix: " << camera_matrix_ << std::endl;
    std::cout << "distortion_coefficients: " << distortion_coefficients_ << std::endl;
    std::cout << "image_center: " << image_center_ << std::endl;

    pnp_solver_ = std::make_shared<PnpSolver>();
    pnp_solver_->set_matrix(camera_width_, camera_height_, camera_matrix_, distortion_coefficients_);
    camera_info_sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Finished receive camera info successfully!");
}

void ArmorDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    detector_end = std::chrono::steady_clock::now();

    std::chrono::duration<double> detector_diff = detector_end - detector_start;

    if(detector_diff.count() >= 1)
    {
        std::cout << detector_diff.count() << "s and detector receive fps: " << detector_fps<< std::endl;
        detector_now_fps = detector_fps;
        detector_start = std::chrono::steady_clock::now();
        detector_fps = 0;
    }

    detector_fps++;

    // 1. Convert ROS image message to OpenCV image
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // 2. Detect armors
    if(openvino_detector_ != nullptr)
    {
        openvino_detector_->infer(image, detect_color_);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Openvino detector is nullptr!");
    }

    // 3. Updata robots
    get_robots(decision_armors_, openvino_detector_->armors_);
    allrobots_adjust(decision_armors_);

    // 4. Decide which armor to shoot
    DecisionArmor decision_armor;
    decision_armor.color = 2;
    decision_armor = decide_armor_shoot(decision_armors_);

    if(decision_armor.color != 2)
    {
        // 5. Solve PnP
        if(pnp_solver_ != nullptr)
        {
            cv::Mat rvec, tvec;
            bool success = pnp_solver_->solve_pnp(decision_armor.four_points, rvec, tvec, decision_armor.is_big_armor);
            if(success == true)
            {
                decision_armor.distance = std::sqrt(tvec.at<double>(0) * tvec.at<double>(0) + tvec.at<double>(1) * tvec.at<double>(1) + tvec.at<double>(2) * tvec.at<double>(2));

                rm_msgs::msg::Armor armor_msg;
                armor_msg.header = msg->header;

                // rvec to 3x3 rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvec, rotation_matrix);
                // rotation matrix to quaternion
                tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
                tf2::Quaternion tf2_q;
                tf2_rotation_matrix.getRotation(tf2_q);
                armor_msg.pose.orientation = tf2::toMsg(tf2_q);

                // tvec to translation
                armor_msg.pose.position.x = tvec.at<double>(0);
                armor_msg.pose.position.y = tvec.at<double>(1);
                armor_msg.pose.position.z = tvec.at<double>(2);

                armor_msg.id = decision_armor.id;
                armor_msg.color = decision_armor.color;
                if(armor_msg.id == 0)
                {
                    armor_msg.name = "sentry";
                }
                else if(armor_msg.id >= 1 && armor_msg.id <= 5)
                {
                    armor_msg.name = std::to_string(armor_msg.id);
                }
                else if(armor_msg.id == 6)
                {
                    armor_msg.name = "outpost";
                }
                else if(armor_msg.id == 7)
                {
                    armor_msg.name = "base";
                }
                else if(armor_msg.id == 8)
                {
                    armor_msg.name = "base_big";
                }
                decision_armor.is_big_armor == true ? armor_msg.type = "BIG" : armor_msg.type = "SMALL";
                armor_msg.distance_to_image_center = decision_armor.distance_to_image_center;
                armor_pub_->publish(armor_msg);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Pnp solver failed!");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pnp solver is nullptr!");
        }
    }
    else // 未识别到有效装甲板，也需要发布消息，刷新tracker
    {
        rm_msgs::msg::Armor armor_msg;
        armor_msg.header = msg->header;
        armor_msg.id = 100;
        armor_msg.color = 2;
        armor_msg.name = "none";
        armor_msg.type = "none";
        armor_msg.distance_to_image_center = 0;
        armor_pub_->publish(armor_msg);
    }
    if(is_debug_ == true)
    {
        debug_deal(image, openvino_detector_->armors_, decision_armor);
    }
}

void ArmorDetectorNode::test()
{
    std::string model_path = "/home/zyicome/zyb/qianli_auto_aim/src/rm_armor_detector/model/four_points_armor/armor.onnx";
    //std::string model_path = "/home/zyicome/zyb/qianli_auto_aim/src/rm_armor_detector/model/inference_armor/armor.onnx";
    OpenvinoDetector detector;
    detector.set_onnx_model(model_path, "CPU");
    cv::Mat input = cv::imread("/home/zyicome/zyb/pictures/armors/images/8.jpg");
    detector.infer(input, detect_color_);
    if(detector.armors_.size() > 0)
    {
        for(size_t i =0;i<detector.armors_.size();i++)
        {
            // Convert vector<cv::Point2f> to vector<vector<cv::Point>>
            std::vector<std::vector<cv::Point>> pts(1);
            for (const auto& p : detector.armors_[i].four_points) {
                pts[0].emplace_back(cv::Point(p.x, p.y));
            }
            cv::polylines(input, pts, true, cv::Scalar(0, 255, 0), 2);
            cv::putText(input, detector.armors_[i].name, cv::Point(detector.armors_[i].rect.x, detector.armors_[i].rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 5);
        }
        cv::imshow("result", input);
        cv::waitKey(0);
    }
    else
    {
        std::cout << "No armor detected!" << std::endl;
    }

}

//-----------------------------------------------------------
void ArmorDetectorNode::robots_init()
{
    RCLCPP_INFO(this->get_logger(), "Begin to init robots!");
    DecisionArmor decision_armor;
    decision_armor.is_big_armor = false;
    decision_armor.is_ignored = false;
    decision_armor.is_continue = false;
    decision_armor.id = 0;
    decision_armor.color = 2; //默认为灰色，即无效装甲板
    decision_armor.priority = 100;
    decision_armor.distance = 0;
    decision_armor.distance_to_image_center = 0;
    decision_armor.four_points = {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)};
    for(int i = 0;i < 9; i++)
    {
        decision_armors_.push_back(decision_armor);
        decision_armor.id++;
    }
    decision_armors_[1].is_big_armor = true; // 英雄必定为大装甲板
    RCLCPP_INFO(this->get_logger(), "Finished init robots successfully!");
}

void ArmorDetectorNode::get_robots(std::vector<DecisionArmor> &decision_armors, const std::vector<Armor> &armors)
{
    bool is_big_armor;
    int armor_id;
    int armor_color;
    std::vector<cv::Point2f> armor_four_points;

    for(size_t i = 0; i<armors.size();i++)
    {
        armor_id = armors[i].id;
        armor_color = armors[i].color;
        armor_four_points = armors[i].four_points;
        is_big_armor = get_is_big_armor(armor_four_points);
        if(get_is_ignored(armors[i], armor_four_points) == true)
        {
            continue;
        }
        decision_armors[armor_id].is_big_armor = is_big_armor;
        decision_armors[armor_id].is_ignored = false;
        decision_armors[armor_id].is_continue = true;
        decision_armors[armor_id].id = armor_id;
        decision_armors[armor_id].color = armor_color;
        cv::Point armor_center = cv::Point((armor_four_points[0].x + armor_four_points[2].x) / 2, (armor_four_points[0].y + armor_four_points[2].y) / 2);
        decision_armors[armor_id].distance_to_image_center = pnp_solver_->get_distance_armor_center_to_image_center(armor_center);
        decision_armors[armor_id].four_points = armor_four_points;
    }
}

void ArmorDetectorNode::allrobots_adjust(std::vector<DecisionArmor> &decision_armors)
{
    for(size_t i =0;i<decision_armors.size();i++)
    {
        if(decision_armors[i].is_continue == false)
        {
            decision_armors[i].color = 2; //默认为灰色，即无效装甲板
            decision_armors[i].distance = 0;
            decision_armors[i].distance_to_image_center = 0;
            decision_armors[i].four_points = {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)};
        }

        decision_armors[i].is_continue = false;
    }
}

bool ArmorDetectorNode::get_is_big_armor(const std::vector<cv::Point2f> &four_points)
{
    double width = cv::norm(four_points[0] - four_points[1]);
    double height = cv::norm(four_points[1] - four_points[2]);
    double ratio = width / height;
    if(ratio > MIN_BIG_ARMOR_RATIO_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ArmorDetectorNode::get_is_ignored(const Armor &armor, const std::vector<cv::Point2f> &four_points)
{
    // 忽略类别判断
    if(ignore_armors_.size() != 0)
    {
        for(size_t i = 0; i<ignore_armors_.size(); i++)
        {
            if(armor.id == ignore_armors_[i])
            {
                return true;
            }
        }
    }
    double width = cv::norm(four_points[0] - four_points[1]);
    if(width == 0)
    {
        return true;
    }
    return false;
}

DecisionArmor ArmorDetectorNode::decide_armor_shoot(const std::vector<DecisionArmor> &decision_armors)
{
    DecisionArmor decision_armor;
    int min_priority = 100;
    int min_distance_to_image_center = 10000;
    int decision_distance_id = 0;
    int decision_priority_id = 0;
    std::vector<int> decision_ids;
    for(size_t i = 0;i<decision_armors.size();i++)
    {
        if(decision_armors[i].color != 2) // 表明该装甲板有效
        {
            if(decision_armors[i].priority < min_priority)
            {
                min_priority = decision_armors[i].priority;
                decision_priority_id = i;
            }
            else if(decision_armors[i].priority == min_priority && min_priority != 100)
            {
                decision_ids.push_back(decision_priority_id); //多插入不影响
                decision_ids.push_back(i);
            }
            if(decision_armors[i].distance_to_image_center < min_distance_to_image_center)
            {
                min_distance_to_image_center = decision_armors[i].distance_to_image_center;
                decision_distance_id = i;
            }
        }
    }
    if(decision_ids.size() != 0)
    {
        min_distance_to_image_center = 10000;
        decision_distance_id = 0;
        for(size_t i = 0;i<decision_ids.size();i++)
        {
            if(decision_armors[decision_ids[i]].distance_to_image_center < min_distance_to_image_center)
            {
                min_distance_to_image_center = decision_armors[decision_ids[i]].distance_to_image_center;
                decision_distance_id = decision_ids[i];
            }
        }
        decision_armor = decision_armors[decision_distance_id];
    }
    else if(min_priority != 100)
    {
        decision_armor = decision_armors[decision_priority_id];
    }
    else
    {
        decision_armor = decision_armors[decision_distance_id];
    }
    return decision_armor;
}

}  // namespace rm_armor_detector

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_detector::ArmorDetectorNode)