/**
 * @file motion-plan.cpp
 * @author 肖书奇
 * @brief HL机器人“运动规划类”的实现
 * @version 0.1
 * @date 2021-05-31
 * 
 */

#include "motion-plan.h"

MotionPlan::MotionPlan(const std::string &cam_param_path,
                       const double &l1,
                       const double &l2,
                       const double &l3,
                       const double &l4) : RigidBodyTransformation::RigidBodyTransformation(cam_param_path, l1, l2, l3, l4)
{
}

MotionPlan::~MotionPlan()
{
}

void MotionPlan::Config(const double *rpy_coor_begin,
                        const double *rpy_coor_end,
                        const double &sampling_time_interval,
                        const double &vel,
                        const double &acc,
                        const double &dec,
                        const std::string &path_type,
                        const std::string &trajectory_type)
{
    for (size_t i = 0; i < 6; i++)
    {
        this->rpy_coor_begin_[i] = rpy_coor_begin[i];
        this->rpy_coor_end_[i] = rpy_coor_end[i];
    }
    this->sampling_time_interval_ = sampling_time_interval;
    this->vel_ = vel;
    this->acc_ = acc;
    this->dec_ = dec;
    this->path_type_ = path_type;
    this->trajectory_type_ = trajectory_type;
}

void MotionPlan::GenerateJointPointsFile(const std::string &joint_pts_file_path)
{
    this->joint_pts_file_path_ = joint_pts_file_path;
    std::ofstream pts_file;
    pts_file.open(this->joint_pts_file_path_, std::ios::out);

    Eigen::VectorXd rpy_coor_deviation(6);
    rpy_coor_deviation << this->rpy_coor_end_[0] - this->rpy_coor_begin_[0],
        this->rpy_coor_end_[1] - this->rpy_coor_begin_[1],
        this->rpy_coor_end_[2] - this->rpy_coor_begin_[2],
        this->rpy_coor_end_[3] - this->rpy_coor_begin_[3],
        this->rpy_coor_end_[4] - this->rpy_coor_begin_[4],
        this->rpy_coor_end_[5] - this->rpy_coor_begin_[5];
    Eigen::VectorXd normalized_rpy_coor_deviation(6);
    double dist = rpy_coor_deviation.norm();
    normalized_rpy_coor_deviation = rpy_coor_deviation / dist;

    double min_dist = (this->vel_ * this->vel_) / (2 * this->acc_) + (this->vel_ * this->vel_) / (2 * this->dec_);

    double t_acc_end, t_dec_begin, t_end;
    if (dist < min_dist)
    {
        std::cout << "Triangle trajectory!" << std::endl;
        t_acc_end = t_dec_begin = std::sqrt(2 * dist * this->dec_ / (this->acc_ * this->dec_ + (this->acc_ * this->acc_)));
        t_end = t_dec_begin + (this->acc_ * t_acc_end) / this->dec_;
    }
    else
    {
        t_acc_end = this->vel_ / this->acc_;
        t_dec_begin = t_acc_end + (dist - min_dist) / this->vel_;
        t_end = t_dec_begin + this->vel_ / this->dec_;
    }

    Eigen::VectorXd tmp_rpy_coor_vec6d(6);
    tmp_rpy_coor_vec6d << this->rpy_coor_begin_[0],
        this->rpy_coor_begin_[1],
        this->rpy_coor_begin_[2],
        this->rpy_coor_begin_[3],
        this->rpy_coor_begin_[4],
        this->rpy_coor_begin_[5];
    double tmp_rpy_coor[6]{0};
    double tmp_joint_coor[6]{0};

    for (double t = 0; t < t_end; t += this->sampling_time_interval_)
    {
        if (t < t_acc_end)
        {
            // tmp_rpy_coor_vec6d += normalized_rpy_coor_deviation * this->acc_ * (2 * t - this->sampling_time_interval_) * this->sampling_time_interval_ / 2;
            tmp_rpy_coor_vec6d += normalized_rpy_coor_deviation * 0.5 * this->acc_ * (2 * t + this->sampling_time_interval_) * this->sampling_time_interval_;
        }
        else if (t > t_acc_end && t < t_dec_begin)
        {
            tmp_rpy_coor_vec6d += normalized_rpy_coor_deviation * this->vel_ * this->sampling_time_interval_;
        }
        else
        {
            // tmp_rpy_coor_vec6d += normalized_rpy_coor_deviation * this->dec_ * (2 * (t_end - t) + this->sampling_time_interval_) * this->sampling_time_interval_ / 2;
            tmp_rpy_coor_vec6d += normalized_rpy_coor_deviation * 0.5 * this->dec_ * (2 * t + this->sampling_time_interval_) * this->sampling_time_interval_;
        }
        for (size_t i = 0; i < 6; i++)
        {
            tmp_rpy_coor[i] = tmp_rpy_coor_vec6d[i];
        }
        this->World2Joint(tmp_rpy_coor, tmp_joint_coor);
        pts_file << tmp_joint_coor[0] << " "
                 << tmp_joint_coor[1] << " "
                 << tmp_joint_coor[2] << " "
                 << tmp_joint_coor[3] << " "
                 << tmp_joint_coor[4] << " "
                 << tmp_joint_coor[5] << " "
                 << std::endl;
    }
    pts_file.close();
}

void MotionPlan::Simulate(const std::string &rpy_pts_file_path)
{
    std::ifstream joint_pts_file;
    std::ofstream rpy_pts_file;
    joint_pts_file.open(this->joint_pts_file_path_, std::ios::in);
    rpy_pts_file.open(rpy_pts_file_path, std::ios::out);

    double tmp_joint_coor[6];
    double tmp_rpy_coor[6];

    std::string raw_str;
    while (std::getline(joint_pts_file, raw_str))
    {
        // std::cout << raw_str << std::endl;
        std::stringstream input(raw_str); // segment
        std::string segmented_str;
        size_t i = 0;
        while (input >> segmented_str)
        {
            tmp_joint_coor[i] = std::stod(segmented_str);
            i++;
        }
        this->Joint2World(tmp_joint_coor, tmp_rpy_coor);
        rpy_pts_file << tmp_rpy_coor[0] << " "
                     << tmp_rpy_coor[1] << " "
                     << tmp_rpy_coor[2] << " "
                     << tmp_rpy_coor[3] << " "
                     << tmp_rpy_coor[4] << " "
                     << tmp_rpy_coor[5] << " "
                     << std::endl;
    }
    joint_pts_file.close();
    rpy_pts_file.close();
}
