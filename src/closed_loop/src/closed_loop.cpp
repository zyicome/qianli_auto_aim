// 2024.10.11初步思考：
// 统一采用odom坐标系进行弹丸坐标的计算 -- 因为odom坐标系是固定的，而其他坐标系随云台而发生变化
// 将每一帧的图像保存下来，用于后续的闭环检测，主要存储时间戳信息，并定义一个初始时间，用于计算时间差，减少时间长度 // 时间ms单位
// 使用自定义map队列，用于存储图像信息，当图像数量超过一定数量时，删除最早的图像信息，将新的图像信息插入队列，保存一定时间内的图像信息，减少内存占用，加快检索
// 主要解算信息：装甲板在图像时间戳下的坐标（目标）| 发射点对于图像时间戳下打击的装甲板的坐标（起点），发射时间，发射速度，发射角度，飞行时间 -- 用于识别的弹丸的模拟弹道解算
// 对于每一帧图片，制作模拟弹丸轨迹的可视化，定时间处理（timer处理）or其他时间处理方法？超过飞行时间的弹丸轨迹不再显示
// 如何闭环？
// 难点：只能获得弹丸在二维图像的像素坐标，而不能获得弹丸在三维空间的坐标，如何将二维坐标转换为三维坐标？ --》关键：弹丸小！！！
// 比对弹丸在二维图像中的坐标和模拟弹道此时弹丸位置坐标的接近程度来判断符合哪一条模拟弹道？ --》 如果能准确匹配的话，就可以再通过模拟弹道和实际弹丸的误差来进行闭环检测

#include "closed_loop.hpp"

ClosedLoop::ClosedLoop()
{
    std::cout << "ClosedLoop constructed" << std::endl;
    parameters_init();

}

void ClosedLoop::parameters_init()
{

}

ClosedLoop::update_tf2_buffer(const std::shared_ptr<tf2_ros::Buffer>& tf2_buffer, const std::shared_ptr<tf2_ros::TransformListener>& tf2_listener)
{
    tf2_buffer_ = tf2_buffer;
    tf2_listener_ = tf2_listener;
}

void ClosedLoop::update_projectiles_messages(const Looper& looper)
{
    int64_t time = looper.time;
    all_projectiles_messages_->insert(time, looper); // insert自动有满序列删除最早的功能
}

geometry_msgs::msg::PoseStamped ClosedLoop::get_projectile_pose(const geometry_msgs::msg::PoseStamped& odom_projectile_pose, const geometry_msgs::msg::PoseStamped& odom_armor_pose, const double& time, const double& v0, const double& theta)
{
    double vx = v0 * std::cos(theta);
    double vy = v0 * std::sin(theta);
    double d_time = time;
    double x;
    double y;
    double v;
    double ax;
    double ay;
    while(d_time >= 0)
    {
        v = std::sqrt(vx * vx + vy * vy);
        Fd = 0.5 * Cd * rho * A * v * v;
        ax = - Fd * vx / v / m;
        ay = - Fd * vy / v / m - g;

        vx += ax * dt;
        vy += ay * dt;
        x += vx * dt;
        y += vy * dt;
        d_time -= dt;
    }

    cv::Mat xy_vec = (cv::Mat_<double>(2, 1) << (odom_armor_pose.pose.position.x - odom_projectile_pose.pose.position.x),(odom_armor_pose.pose.position.y - odom_projectile_pose.pose.position.y));
    xy_vec = cv::normalize(xy_vec);
    geometry_msgs::msg::PoseStamped result;
    result.pose.position.x = odom_armor_pose.pose.position.x + x * xy_vec.at<double>(0, 0);
    result.pose.position.y = odom_armor_pose.pose.position.y + x * xy_vec.at<double>(1, 0);
    result.pose.position.z = odom_armor_pose.pose.position.z + y;
    result.pose.orientation = odom_armor_pose.pose.orientation;
    return result;
}