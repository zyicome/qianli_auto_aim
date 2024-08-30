#include <iostream>

#include "cv_bridge/cv_bridge.h"

#include "image_transport/image_transport.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_msgs/msg/armors.hpp"
#include "rm_msgs/msg/armor.hpp"

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
    float distance_to_image_center;
    std::vector<cv::Point2f> four_points;
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode(const rclcpp::NodeOptions & options);
    void parameters_init();
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void test();

    void robots_init();
    void get_robots(std::vector<DecisionArmor> &decision_armors, const std::vector<Armor> &armors);
    void allrobots_adjust(std::vector<DecisionArmor> &decision_armors);
    bool get_is_big_armor(const std::vector<cv::Point2f> &four_points);
    bool get_is_ignored(const std::vector<cv::Point2f> &four_points);
    DecisionArmor decide_armor_shoot(const std::vector<DecisionArmor> &decision_armors);

    int is_debug_;
    int detect_color_;
    double MIN_BIG_ARMOR_RATIO_;

    int camera_width_;
    int camera_height_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    cv::Point2f image_center_;

    std::vector<DecisionArmor> decision_armors_;

    std::shared_ptr<OpenvinoDetector> openvino_detector_;
    std::shared_ptr<PnpSolver> pnp_solver_;

    rclcpp::Publisher<rm_msgs::msg::Armor>::SharedPtr armor_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    //-------------------------------------------------------------------------------------------
    // debug
    void create_debug_publishers();
    void destroy_debug_publishers();
    void debug_deal(const cv::Mat &image, const std::vector<Armor> &armors, const DecisionArmor &decision_armor);
    
    image_transport::Publisher result_image_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

    //------------------------------------------------------------------------------
    std::chrono::steady_clock::time_point detector_start;
    std::chrono::steady_clock::time_point detector_end;
    int detector_fps;
    int detector_now_fps;
};

}  // namespace rm_armor_detector