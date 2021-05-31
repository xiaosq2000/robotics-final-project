/**
 * @file motion-plan.h
 * @author 肖书奇
 * @brief HL机器人“梯形运动规划类”的声明
 * @version 0.1
 * @date 2021-05-31
 * 
 */

#ifndef _MOTION_PLAN_H_
#define _MOTION_PLAN_H_

#include <rigid-body-transformation.h>
#include <fstream>

class MotionPlan : private RigidBodyTransformation
{
private:
    std::string trajectory_type_;
    std::string path_type_;

    double rpy_coor_begin_[6];
    double rpy_coor_end_[6];
    double sampling_time_interval_;
    double vel_;
    double acc_;
    double dec_;
    std::string joint_pts_file_path_;

public:
    MotionPlan(const std::string &cam_param_path = "../share/eye-in-hand-calibration/dst",
               const double &l1 = 0.491,
               const double &l2 = 0.45,
               const double &l3 = 0.45,
               const double &l4 = 0.084);
    ~MotionPlan();
    void Config(const double *rpy_coor_begin,
                const double *rpy_coor_end,
                const double &sampling_time_interval,
                const double &vel,
                const double &acc,
                const double &dec,
                const std::string &path_type = "line",
                const std::string &trajectory_type = "trapezoid");
    void GenerateJointPointsFile(const std::string &joint_pts_file_path);
    void Simulate(const std::string &rpy_pts_file_path);
};

#endif
