#include "pnp_solver.hpp"

PnpSolver::PnpSolver()
{
    init();
    std::cout << "PnpSolver constructed!" << std::endl;
}

void PnpSolver::init()
{
    // 单位为m
    double small_armor_half_width = SMALL_ARMOR_WIDTH_ / 2;
    double small_armor_half_height = SMALL_ARMOR_LIGHT_HEIGHT_ / 2;
    double big_armor_half_width = BIG_ARMOR_WIDTH_ / 2;
    double big_armor_half_height = BIG_ARMOR_LIGHT_HEIGHT_ / 2;

    // 装甲板实际四个点的坐标，以中心为原点，朝装甲板向外为x轴正方向，单位为m， 左灯条上顶点为起始点，顺时针
    small_armor_points_3d_.push_back(cv::Point3f(0, -small_armor_half_width, small_armor_half_height));
    small_armor_points_3d_.push_back(cv::Point3f(0, small_armor_half_width, small_armor_half_height));
    small_armor_points_3d_.push_back(cv::Point3f(0, small_armor_half_width, -small_armor_half_height));
    small_armor_points_3d_.push_back(cv::Point3f(0, -small_armor_half_width, -small_armor_half_height));

    big_armor_points_3d_.push_back(cv::Point3f(0, -big_armor_half_width, big_armor_half_height));
    big_armor_points_3d_.push_back(cv::Point3f(0, big_armor_half_width, big_armor_half_height));
    big_armor_points_3d_.push_back(cv::Point3f(0, big_armor_half_width, -big_armor_half_height));
    big_armor_points_3d_.push_back(cv::Point3f(0, -big_armor_half_width, -big_armor_half_height));
}

void PnpSolver::set_matrix(const int &image_width, const int &image_height, cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
{
    this->image_width_ = image_width;
    this->image_height_ = image_height;
    this->image_center_ = cv::Point2f(image_width / 2, image_height / 2);
    this->camera_matrix_ = camera_matrix;
    this->dist_coeffs_ = dist_coeffs;
    std::cout << "PnpSolver set matrix successfully!" << std::endl;
}

bool PnpSolver::solve_pnp(const std::vector<cv::Point2f> &points, const cv::Mat &rvec, const cv::Mat &tvec, bool is_big_armor)
{
    // 模型准确度有待测试
    //cv::Mat inliers;
    //cv::solvePnPRansac(points, armor_points_3d_, camera_matrix_, dist_coeffs_, rvec, tvec, true, 100, 8.0, 0.99, inliers);
    if(is_big_armor == false)
    {
        cv::solvePnP(points, small_armor_points_3d_, camera_matrix_, dist_coeffs_, rvec, tvec);
        return true;
    }
    else if(is_big_armor == true)
    {
        cv::solvePnP(points, big_armor_points_3d_, camera_matrix_, dist_coeffs_, rvec, tvec);
        return true;
    }
    else
    {
        return false;
    }
}

double PnpSolver::get_distance_armor_center_to_image_center(const cv::Point2f &armor_center)
{
    // 单位：像素
    double distance_to_image_center = cv::norm(armor_center - image_center_);
    return distance_to_image_center;
}
