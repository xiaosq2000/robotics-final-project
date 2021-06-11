
/**
 * TODO: error: std::stod() out of range
 * TODO: "line" and "Jump" path 
 */

/**
 * @file brick-construction.cpp
 * @author 肖书奇
 * @brief “积木类”与“搭建类”的实现
 * @version 1.0
 * @date 2021-06-01
 * 
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "brick-construction.h"
#include "geometric-median.h"

/**
 * @brief Construct a new Brick:: Brick object
 * 
 * @param pixel_coor 
 * @param angle 
 * 
 */
Brick::Brick(Eigen::Vector2d pixel_coor, double angle)
{
    this->pixel_coor_ = pixel_coor;
    this->angle_ = angle;
    this->src_rpy_coor_.resize(6);
    this->src_rpy_coor_ << Eigen::VectorXd::Zero(6);
    this->dst_rpy_coor_.resize(6);
    this->dst_rpy_coor_ << Eigen::VectorXd::Zero(6);
}

/**
 * @brief Destroy the Brick:: Brick object
 * 
 * 
 */
Brick::~Brick()
{
}

/**
 * @brief Construct a new Construction:: Construction object
 * 
 * @param rpy_tool_config_src 
 * @param construction_configuration 
 * @param img_src_directory 
 * @param img_dst_directory 
 * @param brick_length 
 * @param brick_width 
 * @param brick_height 
 * 
 */
Construction::Construction(const std::string &rpy_tool_config_src_path,
                           const std::string &construction_configuration,
                           const std::string &img_src_directory,
                           const std::string &img_dst_directory,
                           const double &brick_length,
                           const double &brick_width,
                           const double &brick_height)
{
    std::ifstream rpy_config_src_file;
    rpy_config_src_file.open(rpy_tool_config_src_path, std::ios::in);
    if (!rpy_config_src_file)
    {
        std::cout << "File open error!" << std::endl;
    }
    std::string raw_str, segmented_str;
    std::getline(rpy_config_src_file, raw_str);
    std::stringstream input(raw_str);
    size_t i=0;
    while (input >> segmented_str)
    {
        this->rpy_tool_config_src_[i] = std::stod(segmented_str);
        i++;
    }

    this->construction_configuration_ = construction_configuration;
    this->img_src_directory_ = img_src_directory;
    this->img_dst_directory_ = img_dst_directory;
    this->img_src = cv::imread(this->img_src_directory_ + "/img_src.jpg");
    this->img_detection = this->img_src.clone();
    this->img_strategy = this->img_src.clone();
    this->rpy_coor_.resize(6);
    this->rpy_coor_ << Eigen::VectorXd::Zero(6);
    this->brick_length_ = brick_length;
    this->brick_width_ = brick_width;
    this->brick_height_ = brick_height;
}
Construction::~Construction()
{
}

/**
 * @brief HSV segmentation -> Rotated rectangle detection
 * 
 * 
 */
void Construction::BricksDetection()
{
    cv::Mat img_hsv;
    cv::cvtColor(img_src, img_hsv, cv::COLOR_BGR2HSV);
    // HSV segmentation
    int h_min_1 = 0;
    int h_min_2 = 10;
    int s_min = 0;
    int v_min = 224;
    int h_max_1 = 156;
    int h_max_2 = 180;
    int s_max = 255;
    int v_max = 255;
    cv::Scalar hsv_min_1(h_min_1, s_min, v_min);
    cv::Scalar hsv_max_1(h_max_1, s_max, v_max);
    cv::Scalar hsv_min_2(h_min_2, s_min, v_min);
    cv::Scalar hsv_max_2(h_max_2, s_max, v_max);
    cv::Mat img_preprocessed;
    cv::Mat img_preprocessed_1, img_preprocessed_2;
    cv::inRange(img_hsv, hsv_min_1, hsv_max_1, img_preprocessed_1);
    cv::inRange(img_hsv, hsv_min_2, hsv_max_2, img_preprocessed_2);
    img_preprocessed = img_preprocessed_1 + img_preprocessed_2;

    // Rectangle detection
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_preprocessed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> rect(contours.size());
    cv::Point2f vertices[4];

    for (size_t i = 0; i < contours.size(); i++)
    {
        rect[i] = cv::minAreaRect(contours[i]);
        rect[i].points(vertices);
        for (size_t j = 0; j < 4; j++)
        {
            cv::line(this->img_detection, vertices[j], vertices[(j + 1) % 4], cv::Scalar(34, 34, 178), 2);
        }
        // the average of two diagonal lines for each brick
        this->diagonal_pixel_length_ += (norm(cv::Mat(vertices[0]), cv::Mat(vertices[2])) + norm(cv::Mat(vertices[1]), cv::Mat(vertices[3]))) / 2;
        this->bricks_.push_back(Brick(Eigen::Vector2d(static_cast<double>(rect[i].center.x), static_cast<double>(rect[i].center.y)), static_cast<double>(rect[i].angle)));
    }
    this->diagonal_pixel_length_ /= contours.size();
    cv::imwrite(this->img_dst_directory_ + "/img_rectangle_detection.jpg", this->img_detection);
}

/**
 * @brief Function pointers utilized by std::sort()
 * 
 * @param a 
 * @param b 
 * 
 * @return true 
 * @return false 
 */
bool GreaterSort(Brick a, Brick b) { return (a.pixel_dist_construction_center_ > b.pixel_dist_construction_center_); }
bool LessSort(Brick a, Brick b) { return (a.pixel_dist_construction_center_ < b.pixel_dist_construction_center_); }

/**
 * @brief Geometric median -> Arrange -> Display -> Transformation -> Destination poses -> Motion Plan
 * 
 * 
 */
void Construction::Solution()
{

    // location: geometric median with constraints

    std::vector<Eigen::Vector2d> pixel_pts;
    double tmp_angles = 0;
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        pixel_pts.push_back(this->bricks_[i].pixel_coor_);
        tmp_angles += this->bricks_[i].angle_;
    }
    double pixel_min_dist_sum;
    GeometricMedian(pixel_pts, this->pixel_coor_, pixel_min_dist_sum, this->diagonal_pixel_length_, 0.001);

    // Orientation: averge of angles

    this->angle_ = tmp_angles / this->bricks_.size();

    // Label

    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        this->bricks_[i].pixel_dist_construction_center_ = (this->bricks_[i].pixel_coor_ - this->pixel_coor_).norm();
    }
    std::sort(this->bricks_.begin(), this->bricks_.end(), GreaterSort);
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        this->bricks_[i].index_ = i; // Index starts from 0
    }

    // Display

    this->img_strategy = this->img_detection.clone();
    cv::circle(this->img_strategy, cv::Point(this->pixel_coor_(0), this->pixel_coor_(1)), 7, cv::Scalar(230, 224, 176), -1);
    cv::circle(this->img_strategy, cv::Point(this->pixel_coor_(0), this->pixel_coor_(1)), this->diagonal_pixel_length_ / 2, cv::Scalar(230, 224, 176), 1);
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        cv::putText(this->img_strategy, std::to_string(this->bricks_[i].index_ + 1), cv::Point(this->bricks_[i].pixel_coor_(0), this->bricks_[i].pixel_coor_(1)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 1, 8, false);
        cv::line(this->img_strategy, cv::Point(this->pixel_coor_(0), this->pixel_coor_(1)), cv::Point(this->bricks_[i].pixel_coor_(0), this->bricks_[i].pixel_coor_(1)), cv::Scalar(230, 224, 176), 1, 4, 0);
    }
    cv::imwrite(this->img_dst_directory_ + "/img_strategy.jpg", this->img_strategy);

    // Transform pixel information to rpy coordinates in base frame

    TF tf;
    double tmp_pixel_coor[2]{this->pixel_coor_[0], this->pixel_coor_[1]};
    double tmp_camera_coor[3]{0};
    double tmp_tool_coor[3]{0};
    double tmp_rpy_coor[3]{0}; // only xyz
    tf.Pixel2Camera(tmp_pixel_coor, tmp_camera_coor);
    tf.Camera2Tool(tmp_camera_coor, tmp_tool_coor);
    // tf.Tool2World(this->rpy_tool_config_src_, tmp_tool_coor, tmp_rpy_coor);
    this->camera_coor_ << tmp_camera_coor[0], tmp_camera_coor[1], tmp_camera_coor[2];
    this->tool_coor_ << tmp_tool_coor[0], tmp_tool_coor[1], tmp_tool_coor[2];
    // this->rpy_coor_ << tmp_rpy_coor[0], tmp_rpy_coor[1], tmp_rpy_coor[2], this->angle_, 0, 0;
    this->rpy_coor_ << tmp_tool_coor[0], tmp_tool_coor[1], tmp_tool_coor[2], this->angle_, 0, 0;

    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        double tmp_pixel_coor[2]{this->bricks_[i].pixel_coor_[0], this->bricks_[i].pixel_coor_[1]};
        double tmp_camera_coor[3]{0};
        double tmp_tool_coor[3]{0};
        double tmp_rpy_coor[3]{0}; // only xyz
        tf.Pixel2Camera(tmp_pixel_coor, tmp_camera_coor);
        tf.Camera2Tool(tmp_camera_coor, tmp_tool_coor);
        // tf.Tool2World(this->rpy_tool_config_src_, tmp_tool_coor, tmp_rpy_coor);
        this->bricks_[i].camera_coor_ << tmp_camera_coor[0], tmp_camera_coor[1], tmp_camera_coor[2];
        this->bricks_[i].tool_coor_ << tmp_tool_coor[0], tmp_tool_coor[1], tmp_tool_coor[2];
        // this->bricks_[i].src_rpy_coor_ << tmp_rpy_coor[0], tmp_rpy_coor[1], tmp_rpy_coor[2], this->bricks_[i].angle_, 0, 0;
        this->bricks_[i].src_rpy_coor_ << tmp_tool_coor[0], tmp_tool_coor[1], tmp_tool_coor[2], this->bricks_[i].angle_, 0, 0;
    }

    // Compute RPY pose at destination for each brick

    double x, y, z, yaw, pitch, roll = 0;
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        z = this->rpy_coor_[2] + static_cast<int>(i / 4) * this->brick_height_;
        pitch = this->rpy_coor_[4];
        roll = this->rpy_coor_[5];
        switch (i % 4)
        {
        case 0:
            x = this->rpy_coor_[0] - 0.5 * this->brick_length_;
            y = this->rpy_coor_[1];
            yaw = this->rpy_coor_[3];
            this->bricks_[i].dst_rpy_coor_ << x, y, z, yaw, pitch, roll;
            break;
        case 1:
            x = this->rpy_coor_[0] + 0.5 * this->brick_length_;
            y = this->rpy_coor_[1];
            yaw = this->rpy_coor_[3];
            this->bricks_[i].dst_rpy_coor_ << x, y, z, yaw, pitch, roll;
            break;
        case 2:
            x = this->rpy_coor_[0];
            y = this->rpy_coor_[1] - 0.5 * this->brick_length_;
            yaw = this->rpy_coor_[3] + 90;
            if (yaw > 180)
                yaw -= 360;
            this->bricks_[i].dst_rpy_coor_ << x, y, z, yaw, pitch, roll;
            break;
        case 3:
            x = this->rpy_coor_[0];
            y = this->rpy_coor_[1] + 0.5 * this->brick_length_;
            yaw = this->rpy_coor_[3] + 90;
            if (yaw > 180)
                yaw -= 360;
            this->bricks_[i].dst_rpy_coor_ << x, y, z, yaw, pitch, roll;
            break;
        default:
            break;
        }
    }

    // Generate control points

    Eigen::VectorXd tmp_rpy_ctrl_pts = Eigen::VectorXd::Zero(6);
    tmp_rpy_ctrl_pts << this->rpy_tool_config_src_[0],
        this->rpy_tool_config_src_[1],
        this->rpy_tool_config_src_[2],
        this->rpy_tool_config_src_[3],
        this->rpy_tool_config_src_[4],
        this->rpy_tool_config_src_[5];
    this->rpy_ctrl_pts_.push_back(tmp_rpy_ctrl_pts);
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        tmp_rpy_ctrl_pts = this->bricks_[i].src_rpy_coor_;
        this->rpy_ctrl_pts_.push_back(tmp_rpy_ctrl_pts);
        tmp_rpy_ctrl_pts = this->bricks_[i].dst_rpy_coor_;
        this->rpy_ctrl_pts_.push_back(tmp_rpy_ctrl_pts);
    }

    // Motion plan, and generate PPB points file

    MotionPlan motion_plan;
    /*
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        double tmp_rpy_coor_begin[6]{
            this->bricks_[i].src_rpy_coor_[0],
            this->bricks_[i].src_rpy_coor_[1],
            this->bricks_[i].src_rpy_coor_[2],
            this->bricks_[i].src_rpy_coor_[3],
            this->bricks_[i].src_rpy_coor_[4],
            this->bricks_[i].src_rpy_coor_[5],
        };
        double tmp_rpy_coor_end[6]{
            this->bricks_[i].dst_rpy_coor_[0],
            this->bricks_[i].dst_rpy_coor_[1],
            this->bricks_[i].dst_rpy_coor_[2],
            this->bricks_[i].dst_rpy_coor_[3],
            this->bricks_[i].dst_rpy_coor_[4],
            this->bricks_[i].dst_rpy_coor_[5],
        };
        motion_plan.Config(tmp_rpy_coor_begin, tmp_rpy_coor_end, 0.005, 100, 100, -100);
        std::cout << "Index: " << i << std::endl;
        motion_plan.GenerateJointPointsFile("../share/motion-plan/idx_" + std::to_string(i) + ".txt");
        motion_plan.Simulate("../share/motion-plan/simulation/simulation_" + std::to_string(i) + ".txt");
    }
*/
    for (size_t i = 0; i < this->rpy_ctrl_pts_.size() - 1; i++)
    { // .size() - 1
        double tmp_rpy_coor_begin[6]{
            this->rpy_ctrl_pts_[i][0],
            this->rpy_ctrl_pts_[i][1],
            this->rpy_ctrl_pts_[i][2],
            this->rpy_ctrl_pts_[i][3],
            this->rpy_ctrl_pts_[i][4],
            this->rpy_ctrl_pts_[i][5],
        };
        double tmp_rpy_coor_end[6]{
            this->rpy_ctrl_pts_[i + 1][0],
            this->rpy_ctrl_pts_[i + 1][1],
            this->rpy_ctrl_pts_[i + 1][2],
            this->rpy_ctrl_pts_[i + 1][3],
            this->rpy_ctrl_pts_[i + 1][4],
            this->rpy_ctrl_pts_[i + 1][5],
        };
        motion_plan.Config(tmp_rpy_coor_begin, tmp_rpy_coor_end, 0.005, 100, 100, -100);
        std::cout << "Line: " << i + 1 << std::endl;
        if (i < 9)
        {
            motion_plan.GenerateJointPointsFile("../share/motion-plan/line_0" + std::to_string(i + 1) + ".txt");
            // motion_plan.Simulate("../share/motion-plan/simulation/simulation_0" + std::to_string(i + 1) + ".txt");
            if (i != 0) // error: std::stod() out of range
            {
                motion_plan.Simulate("../share/motion-plan/simulation/simulation_0" + std::to_string(i + 1) + ".txt");
            }
        }
        if (i >= 9)
        {
            motion_plan.GenerateJointPointsFile("../share/motion-plan/line_" + std::to_string(i + 1) + ".txt");
            motion_plan.Simulate("../share/motion-plan/simulation/simulation_" + std::to_string(i + 1) + ".txt");
        }
    }
}

/**
 * @brief Write data to file
 * 
 * @param log_path 
 * 
 */
void Construction::Log(const std::string &log_path)
{
    // std::ofstream log_file;
    // log_file.open(log_path, std::ios::ate);

    // log_file << "起始点夹具的位姿(RPY): "
    //          << this->rpy_tool_config_src_[0] << ", "
    //          << this->rpy_tool_config_src_[1] << ", "
    //          << this->rpy_tool_config_src_[2] << ", "
    //          << this->rpy_tool_config_src_[3] << ", "
    //          << this->rpy_tool_config_src_[4] << ", "
    //          << this->rpy_tool_config_src_[5]
    //          << std::endl;
    // log_file << "搭建中心点像素: " << std::endl
    //          << this->pixel_coor_ << std::endl;
    // log_file << "积木塔旋转角度: " << this->angle_ << std::endl;
    // log_file << "相机坐标系下搭建中心点的坐标: " << std::endl
    //          << this->camera_coor_ << std::endl;
    // log_file << "工具坐标系下搭建中心点的坐标: " << std::endl
    //          << this->tool_coor_ << std::endl;
    // log_file << "世界坐标系下搭建中心点的坐标: " << std::endl
    //          << this->rpy_coor_ << std::endl;
    // log_file << std::endl;
    // for (size_t i = 0; i < this->bricks_.size(); i++)
    // {
    //     log_file << "序号: " << this->bricks_[i].index_ << std::endl;
    //     log_file << "起始中心点像素: " << std::endl
    //              << this->bricks_[i].camera_coor_ << std::endl;
    //     log_file << "旋转角度: " << this->bricks_[i].angle_ << std::endl;
    //     log_file << "相机坐标系下起始中心点的坐标: " << std::endl
    //              << this->bricks_[i].camera_coor_ << std::endl;
    //     log_file << "工具坐标系下起始中心点的坐标: " << std::endl
    //              << this->bricks_[i].tool_coor_ << std::endl;
    //     log_file << "世界坐标系下积木的起始位姿(RPY): " << std::endl
    //              << this->bricks_[i].src_rpy_coor_ << std::endl;
    //     log_file << "世界坐标系下积木的终点位姿(RPY): " << std::endl
    //              << this->bricks_[i].dst_rpy_coor_ << std::endl;
    // }
    // log_file.close();

    std::ofstream key_pts_file;
    key_pts_file.open(log_path, std::ios::ate);
    Eigen::IOFormat CleanFmt(4, 0, ", ", " ", "", "");
    for (size_t i = 0; i < this->rpy_ctrl_pts_.size(); i++)
    {
        key_pts_file << this->rpy_ctrl_pts_[i].format(CleanFmt) << std::endl;
    }
    key_pts_file.close();
}
