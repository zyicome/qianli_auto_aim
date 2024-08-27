#include <iostream>

#include "opencv2/opencv.hpp"

#include "openvino/openvino.hpp"

struct Armor
{
    int id; // 0 哨兵 1-5 数字 6 前哨站 7 基地 8 基地大装甲
    int color; // 0: red 1: blue
    float number_score;
    float color_score;
    std::string name;
    std::vector<cv::Point> four_points;
    cv::Rect rect;
};


class OpenvinoDetector
{
public:
    OpenvinoDetector();
    void set_onnx_model(const std::string &model_path, const std::string &device);
    void infer(const cv::Mat &input);
    cv::Mat letterbox(const cv::Mat &input);
    double sigmoid(double x);

    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;

    const float IMAGE_WIDTH = 640;
    const float IMAGE_HEIGHT = 640;
    const float CONFIDENCE_THRESHOLD = 0.5;
    const float SCORE_THRESHOLD = 0.5;
    const float COLOR_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.4;
    const std::vector<std::string> class_names = {"sentry", "1", "2", "3", "4", "5", "outpost", "base", "base_big"};
    std::vector<Armor> armors;
};