#ifndef _CONSTRUCTION_H_
#define _CONSTRUCTION_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "brick.h"
#include "geometric-median.h"

using namespace cv;

bool GreaterSort(Brick a,Brick b);
bool LessSort(Brick a,Brick b);

class Construction
{
private:
    vector<Brick> bricks_;
    Vector2d best_location_camera_;
    double min_dist_camera_;
    double diagonal_length_camera_;

public:
    Construction(const vector<Brick> &bricks, const double &diagonal_length_camera);
    ~Construction();
    void FindBestLocationCamera();
    void Label(bool flag);
    void Display(Mat &img_dst);
    void WriteToFile();
};

#endif