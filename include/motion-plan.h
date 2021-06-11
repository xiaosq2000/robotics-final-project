
/**
 * @file motion-plan.h
 * @author 肖书奇
 * @brief HL机器人“运动规划类”的声明
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#ifndef _MOTION_PLAN_H_
#define _MOTION_PLAN_H_

#include <tf.h>
#include <fstream>

class MotionPlan : private TF
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
               const std::string &src_directory = "../share/sample",
               const double &target_height_offset = 15,
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
