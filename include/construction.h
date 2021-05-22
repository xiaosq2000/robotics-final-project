#ifndef _CONSTRUCTION_H_
#define _CONSTRUCTION_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "brick.h"
#include "MotionPlan.h"
#include "geometric-median.h"

using namespace cv;

bool GreaterSort(Brick a,Brick b);
bool LessSort(Brick a,Brick b);

class Construction
{
private:
    const double kBrickLength_ = 10;
    const double kBrickWidth_ = 10;
    const double kBrickHeight_ = 10;
    vector<Brick> bricks_;
    Vector2d best_location_camera_;
    double min_dist_camera_;
    double diagonal_length_camera_;
    struct PosStruct pose_base_;
public:
    Construction(const vector<Brick> &bricks, const double &diagonal_length_camera);
    ~Construction();
    void FindBestLocationCamera();
    void Label(bool flag);
    void Display(Mat &img_dst);
    void WriteToFile();
    void MotionPlan(const string &bricks_configuration_type = "base", const string &trajectory_type = "trapezoid", const string &path_type = "line", const double &vel = 1, const double &acc = 1, const double &dec = 1);
    
};

#endif