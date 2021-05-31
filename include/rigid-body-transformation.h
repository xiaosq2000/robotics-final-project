/**
 * @file rigid-body-transformation.h
 * @author 肖书奇
 * @brief HL机器人"刚体变换类"的声明
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#ifndef _RIGID_BODY_TRANSFORMATION_H_
#define _RIGID_BODY_TRANSFORMATION_H_

#include <iostream>
#include "Eigen/Dense"
#include <opencv2/core/eigen.hpp>

using namespace Eigen;

Matrix4d Twist2Homo(Vector3d v, Vector3d w, double theta);
Matrix4d Rpy2Homo(double x, double y, double z, double yaw, double pitch, double roll);
void PadenKahanSubproblem1(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double &theta);
void PadenKahanSubproblem2(Vector3d p, Vector3d q, Vector3d r, Vector3d w1, Vector3d w2, double &theta1, double &theta2);
void PadenKahanSubproblem3(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double d, double &theta);

class RigidBodyTransformation
{
private:
    Matrix3d cam_int_param_;
    Matrix4d cam_ext_param_;

    bool config_[3] = {1, 1, 1};
    double link_length_1_;
    double link_length_2_;
    double link_length_3_;
    double link_length_4_;
    Vector3d w[6];
    Vector3d q_[6];

public:
    RigidBodyTransformation(const std::string &cam_param_path = "../share/eye-in-hand-calibration/dst",
                            const double &l1 = 0.491,
                            const double &l2 = 0.45,
                            const double &l3 = 0.45,
                            const double &l4 = 0.084);
    ~RigidBodyTransformation();

    void InverseKinematics(const double *g_vec, double *joint_coor, bool *config);
    void World2Joint(const double *rpy_coor, double *joint_coor);

    void ForwardKinematics(const double *joint_coor, double *g_vec, bool *config);
    void Joint2World(const double *joint_coor, double *rpy_coor);

    void Pixel2Camera(const double *pixel_coor, double *camera_coor);
    void Camera2Tool(const double *camera_coor, double *tool_coor);
};

#endif