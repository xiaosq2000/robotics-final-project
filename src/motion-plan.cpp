/**
 * @file motion-plan.cpp
 * @author 肖书奇
 * @brief HL机器人“运动规划类”的实现
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#include "motion-plan.h"
#include <cmath>

MotionPlan::MotionPlan(const std::string &cam_param_diretory,
                       const std::string &src_directory,
                       const double &target_height_offset,
                       const double &l1,
                       const double &l2,
                       const double &l3,
                       const double &l4) : TF::TF(cam_param_diretory, src_directory, target_height_offset, l1, l2, l3, l4)
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
    std::cout << "==========Planning==========" << std::endl;
    this->joint_pts_file_path_ = joint_pts_file_path;
    std::ofstream pts_file;
    pts_file.open(this->joint_pts_file_path_, std::ios::ate);
    if (!pts_file)
    {
        std::cout << "Fail to open file" << std::endl;
    }
    Eigen::VectorXd rpy_coor_deviation(6);
    rpy_coor_deviation << this->rpy_coor_end_[0] - this->rpy_coor_begin_[0],
        this->rpy_coor_end_[1] - this->rpy_coor_begin_[1],
        this->rpy_coor_end_[2] - this->rpy_coor_begin_[2],
        this->rpy_coor_end_[3] - this->rpy_coor_begin_[3],
        this->rpy_coor_end_[4] - this->rpy_coor_begin_[4],
        this->rpy_coor_end_[5] - this->rpy_coor_begin_[5];
    double dist = rpy_coor_deviation.norm();
    Eigen::VectorXd dir_vec = rpy_coor_deviation / dist;

    std::cout << "distance: " << dist << std::endl;
    std::cout << "unit direction vector: " << std::endl << dir_vec << std::endl;

    if (this->path_type_ == "line" && this->trajectory_type_ == "trapezoid")
    {
        double time_period_acc = this->vel_ / this->acc_;
        double time_period_dec = this->vel_ / this->dec_;
        double time_period_vel = (dist - std::pow(this->vel_, 2) / (2 * this->acc_) - std::pow(this->vel_, 2) / (2 * this->dec_)) / this->vel_;
        double time_period = time_period_acc + time_period_vel + time_period_dec;

        std::cout << "time: " << time_period << ";  acceleration time: " << time_period_acc << ";  max velocity time: " << time_period_vel << ";  deceleration time: " << time_period_dec << std::endl;

        int sampling_pts = static_cast<int>(time_period / this->sampling_time_interval_);
        int sampling_pts_acc_end = static_cast<int>(sampling_pts * time_period_acc / time_period);
        int sampling_pts_dec_begin = static_cast<int>(sampling_pts * (1 - time_period_dec / time_period));

        std::cout << "sampling points: " << sampling_pts << std::endl;
        std::cout <<"the end of acceleration index: " << sampling_pts_acc_end << std::endl;
        std::cout << "the start of decceleration index: " << sampling_pts_dec_begin << std::endl;

        Eigen::VectorXd tmp_rpy_coor(6);
        tmp_rpy_coor << this->rpy_coor_begin_[0],
            this->rpy_coor_begin_[1],
            this->rpy_coor_begin_[2],
            this->rpy_coor_begin_[3],
            this->rpy_coor_begin_[4],
            this->rpy_coor_begin_[5];
        double p_tmp_rpy_coor[6];
        double p_tmp_joint_coor[6];

        for (size_t i = 0; i < sampling_pts - 1; i++)
        {
            if (i <= sampling_pts_acc_end)
            {
                tmp_rpy_coor += 0.5 * dir_vec * this->acc_ * std::pow(this->sampling_time_interval_, 2) * (2 * i + 1);
            }
            else if (i > sampling_pts_acc_end && i <= sampling_pts_dec_begin)
            {
                tmp_rpy_coor += (dir_vec * this->vel_ * this->sampling_time_interval_);
            }
            else
            {
                tmp_rpy_coor += dir_vec * (this->vel_ * this->sampling_time_interval_ - 0.5 * this->dec_ * std::pow(this->sampling_time_interval_, 2) * (2 * i + 1 - 2 * sampling_pts_dec_begin));
            }

            for (size_t j = 0; j < 6; j++)
            {
                p_tmp_rpy_coor[j] = tmp_rpy_coor[j];
            }
            this->World2Joint(p_tmp_rpy_coor, p_tmp_joint_coor);
            pts_file << p_tmp_joint_coor[0] << " "
                     << p_tmp_joint_coor[1] << " "
                     << p_tmp_joint_coor[2] << " "
                     << p_tmp_joint_coor[3] << " "
                     << p_tmp_joint_coor[4] << " "
                     << p_tmp_joint_coor[5] << " "
                     << std::endl;
        }
        pts_file.close();
    }
}

void MotionPlan::Simulate(const std::string &rpy_pts_file_path)
{
    std::ifstream joint_pts_file;
    std::ofstream rpy_pts_file;
    joint_pts_file.open(this->joint_pts_file_path_, std::ios::in);
    rpy_pts_file.open(rpy_pts_file_path, std::ios::ate);

    if (!joint_pts_file || !rpy_pts_file)
    {
        std::cout << "Fail to open file" << std::endl;
    }

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
