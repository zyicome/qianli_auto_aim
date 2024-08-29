#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "openvino_detector.hpp"
#include "pnp_solver.hpp"

namespace rm_armor_detector
{

struct DecisionArmor
{
    bool is_big_armor;
    bool is_ignored;
    bool is_continue;
    int id; // 0 哨兵 1-5 数字 6 前哨站 7 基地 8 基地大装甲
    int color; // 0: red 1: blue
    int priority; // 优先级越高数字越小，优先级高先打，优先级相同则选择里图像中心最近的
    std::vector<cv::Point> four_points;
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode(const rclcpp::NodeOptions & options);
    void parameters_init();
    void test();

    void robots_init();
    void get_robots(std::vector<DecisionArmor> &decision_armors, const std::vector<Armor> &armors);
    void allrobots_adjust(std::vector<DecisionArmor> &decision_armors);
    bool get_is_big_armor(const std::vector<cv::Point> &four_points);
    bool get_is_ignored(const std::vector<cv::Point> &four_points);

    int detect_color_;
    double MIN_BIG_ARMOR_RATIO_;

    std::vector<DecisionArmor> decision_armors_;
};

}  // namespace rm_armor_detector