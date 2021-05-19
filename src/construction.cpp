#include "construction.h"

/**
 * @brief Construct a new Construction:: Construction object
 * 
 * @param bricks 
 */
Construction::Construction(const vector<Brick> &bricks, const double &diagonal_length_camera)
{
    this->bricks_ = bricks;
    this->diagonal_length_camera_ = diagonal_length_camera;
}


/**
 * @brief Find the best location to build bricks in camera frame by solving geometric median with forbidden-zone restriction
 * 
 */
void Construction::FindBestLocationCamera()
{
    vector<Vector2d> points;
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        points.push_back(this->bricks_[i].CenterCamera());
    }
    GeometricMedian(points, this->best_location_camera_, this->min_dist_camera_, 0.01, this->diagonal_length_camera_);
}

/**
 * @brief Print and draw
 * 
 * @param img_dst
 */
void Construction::Display(Mat &img_dst)
{
    cout << "number of bricks: " << this->bricks_.size() << endl;
    cout << "average diagonal length(pixel): " << this->diagonal_length_camera_ << endl;
    cout << "the best location to build blocks(pixel): " << endl
         << this->best_location_camera_ << endl
         << "total distance(pixel): " << endl
         << this->min_dist_camera_<< endl;

    circle(img_dst, Point(this->best_location_camera_(0), this->best_location_camera_(1)), 7, Scalar(230, 224, 176), -1);
    circle(img_dst, Point(this->best_location_camera_(0), this->best_location_camera_(1)), diagonal_length_camera_/2, Scalar(245, 245, 245), 1);
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        putText(img_dst, to_string(i+1), Point(this->bricks_[i].CenterCamera()(0), this->bricks_[i].CenterCamera()(1)), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 215, 255), 1, 8, false);
        line(img_dst, Point(this->best_location_camera_(0), this->best_location_camera_(1)), Point(this->bricks_[i].CenterCamera()(0), this->bricks_[i].CenterCamera()(1)), Scalar(140, 180, 210), 1, 4, 0);
    }
    // putText(img_dst, "(" + to_string(int(this->best_location_camera_(0))) + "," + to_string(int(this->best_location_camera_(1))) + ")", Point(this->best_location_camera_(0), this->best_location_camera_(1)), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 215, 255), 1, 8, false);
}

/**
 * @brief Get the best location in camera frame
 * 
 * @return Vector2d 
 */
// Vector2d Construction::BestLocationCamera()
// {
//     return this->best_location_camera_;
// }

void Construction::Label(bool flag)
{
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        this->bricks_[i].WriteDistCamera((this->bricks_[i].CenterCamera()-best_location_camera_).norm());
    }
    if (flag == true)
    {
        sort(this->bricks_.begin(), this->bricks_.end(), GreaterSort);
    }
    else
    {
        sort(this->bricks_.begin(), this->bricks_.end(), LessSort);
    }
    
    for (size_t i = 0; i < this->bricks_.size(); i++)
    {
        this->bricks_[i].WriteIndex(i);
    }
}

/**
 * @brief 'comp' parameter of std::sort()
 * 
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 */

/**
 * TODO x,idx in brick.h
 * TODO x,idx in brick.cpp 
 * TODO a function to calculate x in construction, .cpp
 * TODO a function to calculate x in construction, .h
 * 
 */

bool GreaterSort (Brick a,Brick b) { return (a.DistCamera()>b.DistCamera()); }
bool LessSort (Brick a,Brick b) { return (a.DistCamera()<b.DistCamera()); }
/**
 * @brief Destroy the Construction:: Construction object
 * 
 */
Construction::~Construction()
{
}