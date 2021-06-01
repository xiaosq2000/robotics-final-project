
/**
 * @file brick-construction.h
 * @author 肖书奇
 * @brief “积木类”与“搭建类”的声明
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#ifndef _BRICK_CONSTRUCTION_H_
#define _BRICK_CONSTRUCTION_H_

#include "tf.h"
#include "motion-plan.h"

class Brick
{
public:
    Eigen::Vector2d pixel_coor_;
    double angle_;
    Eigen::Vector3d camera_coor_;
    Eigen::Vector3d tool_coor_;
    Eigen::VectorXd src_rpy_coor_;
    Eigen::VectorXd dst_rpy_coor_;
    double pixel_dist_construction_center_;
    int index_;

public:
    Brick(Eigen::Vector2d pixel_coor, double angle);
    ~Brick();
};

#endif

class Construction
{
private:
    double rpy_tool_config_src_[6];
    std::string construction_configuration_;
    std::string img_src_directory_;
    std::string img_dst_directory_;

    double brick_length_;
    double brick_width_;
    double brick_height_;

    std::vector<Brick> bricks_;
    Eigen::Vector2d pixel_coor_;
    double angle_;
    Eigen::Vector3d camera_coor_;
    Eigen::Vector3d tool_coor_;
    Eigen::VectorXd rpy_coor_;

    cv::Mat img_src;
    cv::Mat img_detection;
    cv::Mat img_strategy;
    double diagonal_pixel_length_;
public:
    Construction(const double *rpy_tool_config_src,
                 const std::string &construction_configuration = "base",
                 const std::string &img_src_directory = "../share/target-recognition/src",
                 const std::string &img_dst_directory = "../share/target-recognition/dst",
                 const double &brick_length = 500,
                 const double &brick_width = 500,
                 const double &brick_height = 150);
    ~Construction();
    void BricksDetection();
    void Solution();
    void Log(const std::string &log_path = "../share/log.txt");
};
