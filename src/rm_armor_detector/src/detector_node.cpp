#include "detector_node.hpp"

namespace rm_armor_detector
{

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options) : Node("armor_detector_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting ArmorDetectorNode!");
    test();
}

void ArmorDetectorNode::test()
{
    std::string model_path = "/home/zyicome/zyb/qianli_auto_aim/src/rm_armor_detector/model/four_points_armor/armor.onnx";
    //std::string model_path = "/home/zyicome/zyb/qianli_auto_aim/src/rm_armor_detector/model/inference_armor/armor.onnx";
    OpenvinoDetector detector;
    detector.set_onnx_model(model_path, "CPU");
    cv::Mat input = cv::imread("/home/zyicome/zyb/pictures/armors/images/8.jpg");
    detector.infer(input);
    if(detector.armors_.size() > 0)
    {
        for(int i =0;i<detector.armors_.size();i++)
        {
            const cv::Point* pts[1] = { detector.armors_[i].four_points.data() };
            int npts[] = { static_cast<int>(detector.armors_[i].four_points.size()) };
            cv::polylines(input, pts, npts, 1, true, cv::Scalar(0, 255, 0), 2);
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
    DecisionArmor decision_armor;
    decision_armor.is_big_armor = false;
    decision_armor.is_ignored = false;
    decision_armor.is_continue = false;
    decision_armor.id = 0;
    decision_armor.color = 2; //默认为灰色，即无效装甲板
    decision_armor.priority = 100;
    decision_armor.four_points = {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)};
    for(int i = 0;i < 9; i++)
    {
        decision_armors_.push_back(decision_armor);
        decision_armor.id++;
    }
    decision_armors_[1].is_big_armor = true; // 英雄必定为大装甲板
}

void ArmorDetectorNode::get_robots(std::vector<DecisionArmor> &decision_armors, const std::vector<Armor> &armors)
{
    bool is_big_armor;
    int armor_id;
    int armor_color;
    std::vector<cv::Point> armor_four_points;

    for(int i = 0; i<armors.size();i++)
    {
        armor_id = armors[i].id;
        armor_color = armors[i].color;
        armor_four_points = armors[i].four_points;
        is_big_armor = get_is_big_armor(armor_four_points);
        if(get_is_ignored(armor_four_points) == true)
        {
            continue;
        }
        decision_armors[armor_id].is_big_armor = is_big_armor;
        decision_armors[armor_id].is_ignored = false;
        decision_armors[armor_id].is_continue = true;
        decision_armors[armor_id].id = armor_id;
        decision_armors[armor_id].color = armor_color;
        decision_armors[armor_id].four_points = armor_four_points;
    }
}

void ArmorDetectorNode::allrobots_adjust(std::vector<DecisionArmor> &decision_armors)
{
    for(int i =0;i<decision_armors.size();i++)
    {
        if(decision_armors[i].is_continue == false)
        {
            decision_armors[i].color = 2; //默认为灰色，即无效装甲板
            decision_armors[i].four_points = {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)};
        }

        decision_armors[i].is_continue = false;
    }
}

bool ArmorDetectorNode::get_is_big_armor(const std::vector<cv::Point> &four_points)
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

bool ArmorDetectorNode::get_is_ignored(const std::vector<cv::Point> &four_points)
{
    return false;
}

}  // namespace rm_armor_detector

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_detector::ArmorDetectorNode)