#include "tracker_node.hpp"

namespace rm_armor_tracker
{

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : Node("armor_tracker_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting ArmorTrackerNode!");
    parameters_init();
    //test();
}

void ArmorTrackerNode::parameters_init()
{
    this->declare_parameter("tracker_debug", 0);
    is_debug_ = this->get_parameter("tracker_debug").as_int();
    std::cout << "is_tracker_debug: " << is_debug_ << std::endl;
    if(is_debug_ == true)
    {
        create_debug_publishers();
    }
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("tracker_debug", [this](const rclcpp::Parameter & p) {
      is_debug_ = p.as_bool();
      is_debug_ ? create_debug_publishers() : destroy_debug_publishers();
    });

    // Subscriber with tf2 message_filter
    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    armor_sub_.subscribe(this, "/detector/armor", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(
        armor_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(1));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&ArmorTrackerNode::armorCallback, this);

}

void ArmorTrackerNode::armorCallback(const rm_msgs::msg::Armor::SharedPtr armor_msg)
{
    std::cout << "armorCallBack" << std::endl;
    
    

    tracker_->tracker_init(armor_msg);
}

//--------------------------------------------------------------------------------------------------
// debug
void ArmorTrackerNode::create_debug_publishers()
{
    // sub
    detector_result_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/detector_result_image", 10, std::bind(&ArmorTrackerNode::ResultImageCallback, this, std::placeholders::_1));
    
    // pub
    tracker_result_image_pub_ = image_transport::create_publisher(this, "/tracker/result_image");
}

void ArmorTrackerNode::destroy_debug_publishers()
{
    // sub
    detector_result_image_sub_.reset();

    // pub
    tracker_result_image_pub_.shutdown();
}

void ArmorTrackerNode::ResultImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if(is_debug_ == true)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        cv::putText(image, "tracker", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 5);
        cv_bridge::CvImage cv_image;
        cv_image.image = image;
        cv_image.encoding = "bgr8";
        cv_image.header = msg->header;
        tracker_result_image_pub_.publish(cv_image.toImageMsg());
    }
}
//--------------------------------------------------------------------------------------------------


}  // namespace rm_armor_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_tracker::ArmorTrackerNode)