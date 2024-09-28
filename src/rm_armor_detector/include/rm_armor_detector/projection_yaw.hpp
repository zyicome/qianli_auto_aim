#include "opencv2/opencv.hpp"

#include "armor.hpp"

#include <iostream>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/point_stamped.hpp"

class ProjectionYaw
{
public:
    ProjectionYaw();

    double get_cost(const std::vector<cv::Point2f>& pred_points, 
                                const std::vector<cv::Point2f>& points,const double& pred_yaw);

    double get_abs_angle(const cv::Point2f& first_point, const cv::Point2f& second_point);

    double update_pred_yaw(double left_yaw, double right_yaw,const int& ITERATIONS_NUM);

    std::vector<cv::Point3f> big_armor_world_points_;
    std::vector<cv::Point3f> small_armor_world_points_;

    //-----------------------------------------------------------------
    // tf2
    // Subscriber with tf2 message_filter
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    //-----------------------------------------------------------------
};