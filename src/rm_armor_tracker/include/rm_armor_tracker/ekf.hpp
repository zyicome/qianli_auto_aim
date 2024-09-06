#include <iostream>

#include "opencv2/opencv.hpp"

class EKF
{
public:
    EKF();
    void ekf_init(cv::Mat Q_, cv::Mat R_);
    void set_state(cv::Mat &state);
    void motion_model(cv::Mat &state);
    cv::Mat observation_model(cv::Mat &state);
    cv::Mat jacob_f();
    cv::Mat jacob_h(const cv::Mat &state);
    void EKF_predict();
    void EKF_update(const cv::Mat &z);

    double dt_;

    cv::Mat state_;
    cv::Mat P_;
    cv::Mat Q_;
    cv::Mat R_;
};