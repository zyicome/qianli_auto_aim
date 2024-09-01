#include <iostream>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rm_msgs/msg/armor.hpp"

namespace rm_armor_tracker
{

class ArmorTrackerNode : public rclcpp::Node
{
public:
    ArmorTrackerNode(const rclcpp::NodeOptions &options);
    void parameters_init();
};

} // namespace rm_armor_tracker