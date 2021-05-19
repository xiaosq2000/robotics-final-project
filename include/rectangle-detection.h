
#ifndef _RECTANGLE_DETECTION_H_
#define _RECTANGLE_DETECTION_H_

#include <iostream>
#include <vector>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "brick.h"

using namespace cv;

void HsvRedColorSeg(const Mat &src, Mat &dst);
void RectDetection(Mat &src, Mat &dst, vector<Brick> &bricks, double &diagonal_length);

#endif