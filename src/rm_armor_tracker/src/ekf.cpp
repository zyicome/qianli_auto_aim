#include "ekf.hpp"

EKF::EKF()
{
    std::cout << "EKF constructed!" << std::endl;
}

void EKF::ekf_init(cv::Mat Q, cv::Mat R)
{
    std::cout << "EKF init" << std::endl;
    this->Q_ = Q;
    this->R_ = R;
    std::cout << "Finished init EKF" << std::endl;
}

void EKF::set_state(cv::Mat &state)
{
    state_ = state;
}

void EKF::motion_model(cv::Mat &state)
{
    //                                    xc xc_v yc yc_v za za_v yaw yaw_v r
    cv::Mat F = (cv::Mat_<double>(9,9) << 1, dt_, 0, 0, 0, 0, 0, 0, 0,
                                           0, 1, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 1, dt_, 0, 0, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 1, dt_, 0, 0, 0,
                                           0, 0, 0, 0, 0, 1, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 1, dt_, 0,
                                           0, 0, 0, 0, 0, 0, 0, 1, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0, 1);

    state = F * state;                                        
}

cv::Mat EKF::observation_model(cv::Mat &state)
{
    cv::Mat Z = cv::Mat::zeros(4, 1, CV_64F);
    double yaw = state.at<double>(6, 0);
    double r = state.at<double>(8, 0);
    Z.at<double>(0, 0) = state.at<double>(0, 0) - r * cos(yaw);
    Z.at<double>(1, 0) = state.at<double>(2, 0) - r * sin(yaw);
    Z.at<double>(2, 0) = state.at<double>(4, 0);
    Z.at<double>(3, 0) = state.at<double>(6, 0);
    return Z;
}

cv::Mat EKF::jacob_f()
{
    cv::Mat jF = (cv::Mat_<double>(9, 9) << 1, dt_, 0, 0, 0, 0, 0, 0, 0,
                                            0, 1, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 1, dt_, 0, 0, 0, 0, 0,
                                            0, 0, 0, 1, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 1, dt_, 0, 0, 0,
                                            0, 0, 0, 0, 0, 1, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 1, dt_, 0,
                                            0, 0, 0, 0, 0, 0, 0, 1, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 1);
    return jF;
}

cv::Mat EKF::jacob_h(const cv::Mat &state)
{
    double yaw = x.at<double>(6,0);
    double r = x.at<double>(8,0);
    cv::Mat jH;
    jH = (cv::Mat_<double>(4, 9) << 1,   0,   0,   0,   0,   0,   r * sin(yaw),          0,   -cos(yaw),
                                    0,   0,   1,   0,   0,   0,   -r * cos(yaw),          0,   -sin(yaw),
                                    0,   0,   0,   0,   1,   0,   0,          0,   0,
                                    0,   0,   0,   0,   0,   0,   1,          0,   0);
    return jH;  
}

void EKF::EKF_predict()
{
    // 1. Predict
    motion_model(state_);
    cv::Mat jF = jacob_f();
    P_ = jF * P_ * jF.t() + Q_;
}

void EKF::EKF_update(const cv::Mat &z)
{
    // 2. Update
    cv::Mat jH = jacob_h(state_);
    cv::Mat z_pred = observation_model(state_);
    cv::Mat y = z - z_pred;
    cv::Mat S = jH * P_ * jH.t() + R_;
    cv::Mat K = P_ * jH.t() * S.inv(cv::DECOMP_SVD);
    state_ = state_ + K * y;
    P_ = (cv::Mat::eye(state_.rows, state_.rows, CV_64F) - K * jH) * P_;
}