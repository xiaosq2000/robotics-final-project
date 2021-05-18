
#include "brick.h"

Brick::Brick(const Vector2d &center, const double &angle)
{
    this->index = 0;
    this->center_camera_frame_ = center;
    this->angle_camera_frame_ = angle;
    // cout << "A Brick object is created." << endl;
}

Brick::~Brick()
{
    // cout << "A Brick object is deleted." << endl;
}

void Brick::Print()
{
    cout << "center(pixel): " << endl
         << this->center_camera_frame_ << endl
         << "angle: " << endl
         << this->angle_camera_frame_ << endl;
}

bool Brick::WriteToFile()
{
    return false;
}