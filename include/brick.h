
#ifndef _BRICK_H_
#define _BRICK_H_

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

class Brick
{
private:
    const double length_world_frame_ = 10;
    const double width_world_frame_ = 10;
    const double height_world_frame_ = 10;
    int index;
    Vector2d center_camera_frame_;
    double angle_camera_frame_;

public:
    Brick(const Vector2d &center, const double &angle);
    ~Brick();
    void Print();
    bool WriteToFile();
};

#endif
