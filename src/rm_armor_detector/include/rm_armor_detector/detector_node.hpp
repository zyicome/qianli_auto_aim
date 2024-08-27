#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "openvino_detector.hpp"

namespace rm_armor_detector
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode(const rclcpp::NodeOptions & options);
    void test();
};

}  // namespace rm_armor_detector