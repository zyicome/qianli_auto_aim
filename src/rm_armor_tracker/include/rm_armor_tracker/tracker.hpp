#include <iostream>

#include "rm_msgs/msg/armor.hpp"

#include "ekf.hpp"

struct TrackerArmor
{
    int armor_id;
    int armor_num;
    std::string armor_name;
    std::string status;
};

// armor_id: 0, 1, 2, 3, 4, 5, 6, 7, 8
// status: "LOST", "DETECTING","TRACKING", "LOSTING"

class Tracker
{
public:
    Tracker();
    void tracker_init(const rm_msgs::msg::Armor::SharedPtr armor_msg);

    double max_match_distance_;
    double max_match_yaw_diff_;

    double dz_;
    double another_r_;
    double last_yaw_;

    cv::Mat tracking_state_;

    std::unique_ptr<TrackerArmor> tracker_armors_;
    std::shared_ptr<EKF> ekf_;
};