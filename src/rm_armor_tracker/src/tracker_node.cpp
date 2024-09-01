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

}


}  // namespace rm_armor_tracker

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_tracker::ArmorTrackerNode)