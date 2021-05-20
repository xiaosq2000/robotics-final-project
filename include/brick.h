
#ifndef _BRICK_H_
#define _BRICK_H_

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

class Brick
{
private:
    double length_world_;
    double width_world_;
    double height_world_;
    Vector2d center_camera_;
    double angle_camera_;
    double dist_camera_;
    int index_;

public:
    Brick(const Vector2d &center, const double &angle);
    ~Brick();
    void WriteDistCamera(double dist_camera);
    void WriteIndex(int index);
    Vector2d CenterCamera();
    double AngleCamera();
    double DistCamera();
    int Index();
    void Print();
};

#endif
