
#ifndef _BRICK_H_
#define _BRICK_H_

#include <iostream>
#include "Eigen/Dense"
#include "MotionPlan.h"

using namespace std;
using namespace Eigen;

class Brick
{
private:
    Vector2d center_camera_;
    double angle_camera_;

    double dist_camera_;
    int index_;

    struct PosStruct pose_origin_;
    struct PosStruct pose_destination_;

public:
    Brick(const Vector2d &center, const double &angle);
    ~Brick();

    Vector2d CenterCamera();
    double AngleCamera();

    void WriteDistCamera(double dist_camera);
    double DistCamera();
    void WriteIndex(int index);
    int Index();

    void WritePoseOrigin(double x, double y, double z, double yaw, double pitch, double roll);
    struct PosStruct PoseOrigin();
    void WritePoseDestination(double x, double y, double z, double yaw, double pitch, double roll);
    struct PosStruct PoseDestination();

    void Print();
};

#endif
