
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

/**
 * @brief Write the distance between the brick center and the construction site center in camera frame 
 * 
 * @param dist_camera 
 * 
 */
void Brick::WriteDistCamera(double dist_camera)
{
    this->dist_camera_ = dist_camera;
}

/**
 * @brief Get the distance between the brick center and the construction site center in camera frame.
 * 
 * 
 * @return double 
 */
double Brick::DistCamera()
{
    return this->dist_camera_;
}

/**
 * @brief Write the index of the brick, which determines the sequence to be built.
 * 
 * @param index 
 * 
 */
void Brick::WriteIndex(int index)
{
    this->index_ = index;
}

/**
 * @brief Get the index of the brick.
 * 
 * 
 * @return int 
 */
int Brick::Index()
{
    return this->index_;
}

/**
 * @brief Write values into `struct PosStruct pose_destination_`.
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param yaw 
 * @param pitch 
 * @param roll 
 * 
 */
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

/**
 * @brief Write values into `struct PosStruct pose_origin_`, which stores the origin pose of the brick in world frame.
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param yaw 
 * @param pitch 
 * @param roll 
 * 
 */
void Brick::WritePoseOrigin(double x, double y, double z, double yaw, double pitch, double roll)
{
    this->pose_origin_.x = x;
    this->pose_origin_.y = y;
    this->pose_origin_.z = z;
    this->pose_origin_.yaw = yaw;
    this->pose_origin_.pitch = pitch;
    this->pose_origin_.roll = roll;
}

/**
 * @brief Get the origin pose of the brick in world frame.
 * 
 * 
 * @return struct PosStruct 
 */
struct PosStruct Brick::PoseOrigin()
{
    return this->pose_origin_;
}

/**
 * @brief Print
 * 
 * 
 */
void Brick::Print()
{
    cout << "center(pixel): " << endl
         << this->center_camera_ << endl
         << "angle: " << endl
         << this->angle_camera_ << endl
         << "pose in world frame: " << endl
         << this->pose_origin_.x << ", " 
         << this->pose_origin_.y << ", " 
         << this->pose_origin_.z << ", " 
         << this->pose_origin_.yaw << ", " 
         << this->pose_origin_.pitch << ", " 
         << this->pose_origin_.roll << endl;
}
