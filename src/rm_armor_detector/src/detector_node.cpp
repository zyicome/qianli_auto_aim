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
    if(detector.armors.size() > 0)
    {
        for(int i =0;i<detector.armors.size();i++)
        {
            const cv::Point* pts[1] = { detector.armors[i].four_points.data() };
            int npts[] = { static_cast<int>(detector.armors[i].four_points.size()) };
            cv::polylines(input, pts, npts, 1, true, cv::Scalar(0, 255, 0), 2);
            cv::putText(input, detector.armors[i].name, cv::Point(detector.armors[i].rect.x, detector.armors[i].rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 5);
        }
        cv::imshow("result", input);
        cv::waitKey(0);
    }
    else
    {
        std::cout << "No armor detected!" << std::endl;
    }
} 

}  // namespace rm_armor_detector
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_detector::ArmorDetectorNode)