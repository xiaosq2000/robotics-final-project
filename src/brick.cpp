
#include "brick.h"

/**
 * @brief Construct a new Brick:: Brick object
 * 
 * @param center 
 * @param angle 
 */
Brick::Brick(const Vector2d &center, const double &angle)
{
    this->center_camera_ = center;
    this->angle_camera_ = angle;
    this->index_ = 0;
    this->dist_camera_ = 0;
    // cout << "A Brick object is created." << endl;
}

/**
 * @brief Destroy the Brick:: Brick object
 * 
 */
Brick::~Brick()
{
    // cout << "A Brick object is deleted." << endl;
}

/**
 * @brief Get the center point pixel (Vector2d)
 * 
 * @return Vector2d 
 */
Vector2d Brick::CenterCamera()
{
    return this->center_camera_;
}

/**
 * @brief Get the rotation angle of the brick in camera frame (degree)
 * 
 * @return double 
 */
double Brick::AngleCamera()
{
    return this->angle_camera_;
}

void Brick::WriteDistCamera(double dist_camera)
{
    this->dist_camera_ = dist_camera;
}

double Brick::DistCamera()
{
    return this->dist_camera_;
}

void Brick::WriteIndex(int index)
{
    this->index_ = index;
}

int Brick::Index()
{
    return this->index_;
}

void Brick::WritePoseDestination(double x, double y, double z, double yaw, double pitch, double roll)
{
    this->pose_destination_.x = x;
    this->pose_destination_.y = y;
    this->pose_destination_.z = z;
    this->pose_destination_.yaw = yaw;
    this->pose_destination_.pitch = pitch;
    this->pose_destination_.roll = roll;
}

struct PosStruct Brick::PoseDestination()
{
    return this->pose_destination_;
}

void Brick::WritePoseOrigin(double x, double y, double z, double yaw, double pitch, double roll)
{
    this->pose_origin_.x = x;
    this->pose_origin_.y = y;
    this->pose_origin_.z = z;
    this->pose_origin_.yaw = yaw;
    this->pose_origin_.pitch = pitch;
    this->pose_origin_.roll = roll;
}

struct PosStruct Brick::PoseOrigin()
{
    return this->pose_origin_;
}

void Brick::Print()
{
    cout << "center(pixel): " << endl
         << this->center_camera_ << endl
         << "angle: " << endl
         << this->angle_camera_ << endl;
}
