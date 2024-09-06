#include <iostream>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "image_transport/image_transport.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>


#include "rm_msgs/msg/armor.hpp"

#include "tracker.hpp"

namespace rm_armor_tracker
{

using tf2_filter = tf2_ros::MessageFilter<rm_msgs::msg::Armor>;

class ArmorTrackerNode : public rclcpp::Node
{
public:
    ArmorTrackerNode(const rclcpp::NodeOptions &options);
    void parameters_init();
    void armorCallback(const rm_msgs::msg::Armor::SharedPtr armor_msg);

    std::shared_ptr<Tracker> tracker_;

    //-----------------------------------------------------------------
    // tf2
    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<rm_msgs::msg::Armor> armor_sub_;
    std::shared_ptr<tf2_filter> tf2_filter_;
    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
    // debug
    void create_debug_publishers();
    void destroy_debug_publishers();
    void ResultImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    int is_debug_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr detector_result_image_sub_;

    image_transport::Publisher tracker_result_image_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
};

} // namespace rm_armor_tracker