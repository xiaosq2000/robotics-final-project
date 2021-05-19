
#ifndef _BRICK_H_
#define _BRICK_H_

#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

class Brick
{
private:
    double length_world;
    double width_world;
    double height_world;
    Vector2d center_camera_;
    double angle_camera_;
    int index_;
    double dist_camera_;

public:
    Brick(const Vector2d &center, const double &angle);
    ~Brick();
    Vector2d CenterCamera();
    double AngleCamera();
    void Print();
    bool WriteToFile();
    void WriteDistCamera(double dist_camera);
    double DistCamera();
    void WriteIndex(int index);
    int Index();
};

#endif
