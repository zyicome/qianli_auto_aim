#include "armor_ekf.hpp"

ArmorEKF::ArmorEKF()
{
    std::cout << "ArmorEKF constructed!" << std::endl;
}

void ArmorEKF::ekf_init(cv::Mat Q, cv::Mat R)
{
    std::cout << "ArmorEKF init" << std::endl;
    this->Q_ = Q;
    this->R_ = R;
    std::cout << "Finished init ArmorEKF" << std::endl;
}

void ArmorEKF::set_state(cv::Mat &state)
{
    state_ = state;
}

void ArmorEKF::motion_model(cv::Mat &state)
{
    //                                    xa xa_v ya ya_v
    cv::Mat F = (cv::Mat_<double>(4,4) << 1, dt_, 0, 0,
                                           0, 1, 0, 0,
                                           0, 0, 1, dt_,
                                           0, 0, 0, 1);
    state = F * state;                                        
}

void ArmorEKF::observation_model(cv::Mat &state)
{
    cv::Mat Z = cv::Mat::zeros(2, 1, CV_64F);
    Z.at<double>(0, 0) = state.at<double>(0, 0);
    Z.at<double>(1, 0) = state.at<double>(2, 0);
    return Z;
}

cv::Mat ArmorEKF::jacob_f()
{
    cv::Mat jF = (cv::Mat_<double>(4, 4) << 1, dt_, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, dt_,
                                            0, 0, 0, 1);
    return jF;
}

cv::Mat ArmorEKF::jacob_h()
{
    cv::Mat jH = (cv::Mat_<double>(2, 4) << 1, 0, 0, 0,
                                            0, 0, 1, 0);
    return jH;
}

void ArmorEKF::EKF_predict()
{
    motion_model(state_);
    cv::Mat jF = jacob_f();
    P_ = jF * P_ * jF.t() + Q_;
}

void ArmorEKF::EKF_update(cv::Mat &Z)
{
    // 2. Update
    cv::Mat jH = jacob_h();
    cv::Mat z_pred = observation_model(state_);
    cv::Mat y = z - z_pred;
    cv::Mat S = jH * P_ * jH.t() + R_;
    cv::Mat K = P_ * jH.t() * S.inv(cv::DECOMP_SVD);
    state_ = state_ + K * y;
    P_ = (cv::Mat::eye(state_.rows, state_.rows, CV_64F) - K * jH) * P_;
}

    