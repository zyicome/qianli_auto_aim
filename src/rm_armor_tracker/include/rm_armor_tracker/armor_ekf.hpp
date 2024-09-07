#include <iostream>

#include <opencv2/opencv.hpp>

class ArmorEKF
{
public:
    ArmorEKF();
    void ekf_init(cv::Mat Q_, cv::Mat R_);
    void set_state(cv::Mat &state);
    void motion_model(cv::Mat &state);
    cv::Mat observation_model(cv::Mat &state);
    cv::Mat jacob_f();
    void EKF_predict();
    void EKF_update(cv::Mat &Z);

    double dt_;

    cv::Mat state_;
    cv::Mat P_;
    cv::Mat Q_;
    cv::Mat R_;
};