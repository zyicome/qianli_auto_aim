#include "tracker.hpp"

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
{
    std::cout << "Tracker constructed" << std::endl;
    this->max_match_distance_ = max_match_distance;
    this->max_match_yaw_diff_ = max_match_yaw_diff;
    tracker_armors_ = std::make_unique<TrackerArmor>();
    tracker_armors_->armor_id = -1;
    tracker_armors_->armor_num = 0;
    tracker_armors_->armor_name = "";
    tracker_armors_->status = "LOST";
}

void Tracker::tracker_init(const rm_msgs::msg::Armor::SharedPtr armor_msg)
{
    tracker_armors_->armor_id = armor_msg->id;
    if(armor_msg->id == 0)
    {
        tracker_armors_->armor_num = 4;
        tracker_armors_->armor_name = "sentry-4";
    }
    else if(armor_msg->id >= 1 && armor_msg->id <= 5)
    {
        if(armor_msg->type == "BIG")
        {
            tracker_armors_->armor_num = 2;
            tracker_armors_->armor_name = "robot" + std::to_string(armor_msg->id) + "-2";
        }
        else if(armor_msg->type == "SMALL")
        {
            tracker_armors_->armor_num = 4;
            tracker_armors_->armor_name = "robot" + std::to_string(armor_msg->id) + "-4";
        }
    }
    else if(armor_msg->id == 6)
    {
        tracker_armors_->armor_num = 3;
        tracker_armors_->armor_name = "outpost-3";
    }
    else if(armor_msg->id == 7)
    {
        tracker_armors_->armor_num = 1;
        tracker_armors_->armor_name = "base-1";
    }
    else if(armor_msg->id == 8)
    {
        tracker_armors_->armor_num = 1;
        tracker_armors_->armor_name = "base_big-1";
    }
    tracker_armors_->status = "DETECTING";

    EKF_init(armor_msg);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker_node"), "Init EKF!");
}

void EKF_init(const rm_msgs::msg::Armor::SharedPtr &armor_msg)
{
    double xa = armor_msg->pose.position.x;
    double ya = armor_msg->pose.position.y;
    double za = armor_msg->pose.position.z;
    last_yaw = 0;
    double yaw = orientationToYaw(armor_msg->pose.orientation);
    
    cv::Mat state = (cv::Mat_<double>(9,1) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    double r = 0.26;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0.0;
    another_r = 0.0;
    state.at<double>(0,0) = xc;
    state.at<double>(2,0) = yc;
    state.at<double>(4,0) = za;
    state.at<double>(6,0) = yaw;
    state.at<double>(8,0) = r;

    ekf_.set_state(state);
}

void Tracker::tracker_update(const rm_msgs::msg::Armor::SharedPtr armor_msg)
{
    ekf_->EKF_predict();
    tracking_state_ = ekf_->state_;

    bool is_matched = false;

    if(armor_msg->color != 2)
    {
        // Find the closest armor with the same id
        my_msgs::msg::Armor same_id_armor;
        int same_id_armor_count = 0;

    }
}

// 用于将给定的四元数表示的姿态（朝向）转换为偏航角（yaw）
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  // 将当前计算得到的偏航角 yaw 与上一次记录的偏航角 last_yaw_ 进行比较，
  // 使用 angles::shortest_angular_distance 函数计算两者之间的最短角度差，
  // 从而确保偏航角变化在连续范围内。这一步旨在解决角度从 -pi 到 pi 的跳变问题，使角度变化连续。
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

cv::Point3d Tracker::get_armor_position(const cv:Mat &state)
{
    double xc = state.at<double>(0,0);
    double yc = state.at<double>(2,0);
    double za = state.at<double>(4,0);
    double yaw = state.at<double>(6,0);
    double r = state.at<double>(8,0);

    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    return cv::Point3d(xa, ya, za);
}