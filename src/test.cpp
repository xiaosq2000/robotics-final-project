
/* Includes ----------------------------------------*/
#include <iostream>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "Eigen/Dense"

#include "brick.h"
#include "geometric-median.h"
/* Namespace ----------------------------------------*/
using namespace std;
using namespace cv;
using namespace Eigen;
/* Gloabl constant variable ----------------------------------------*/
const string kFileDirPath = "C:/toy-projects/robotics-final-project/robotics-final-project/data/rectangle-detection/";
const double kBrickHeight = 10;

void HsvRedColorSeg(const Mat &src, Mat &dst)
{
    int h_min_1 = 0;
    int h_min_2 = 10;
    int s_min = 0;
    int v_min = 224;
    int h_max_1 = 156;
    int h_max_2 = 180;
    int s_max = 255;
    int v_max = 255;
    Scalar hsv_min_1(h_min_1, s_min, v_min);
    Scalar hsv_max_1(h_max_1, s_max, v_max);
    Scalar hsv_min_2(h_min_2, s_min, v_min);
    Scalar hsv_max_2(h_max_2, s_max, v_max);
    Mat dst_1, dst_2;
    inRange(src, hsv_min_1, hsv_max_1, dst_1);
    inRange(src, hsv_min_2, hsv_max_2, dst_2);
    dst = dst_1 + dst_2;
}

double RectDetection(Mat &src, Mat &dst, vector<Brick> &bricks)
{
    vector<vector<Point>> contours;
    findContours(src, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> rect(contours.size());
    Point2f vertices[4];
    float diagonal_length = 0;
    for (size_t i = 0; i < contours.size(); i++)
    {
        rect[i] = minAreaRect(contours[i]);
        rect[i].points(vertices);
        for (size_t j = 0; j < 4; j++)
        {
            line(dst, vertices[j], vertices[(j + 1) % 4], Scalar(0, 0, 255), 2);
        }
        diagonal_length += (norm(Mat(vertices[0]), Mat(vertices[2])) + norm(Mat(vertices[1]), Mat(vertices[3]))) / 2;
        bricks.push_back(Brick(Vector2d(double(rect[i].center.x), double(rect[i].center.y)), double(rect[i].angle)));
        cout << rect[i].center.x << endl;
        cout << rect[i].center.y << endl;
        // t << rect[i].angle << endl;
    }
    diagonal_length /= contours.size();
    return double(diagonal_length);
}

/**
 * @brief The entry of the test application for rectangular recognition.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    Mat img_src = imread(kFileDirPath + "test.jpg");
    // cout << img_src.size() << endl; // 1280 x 720
    Mat img_hsv = Mat::zeros(img_src.size(), img_src.type());
    Mat img_hsv_seg = Mat::zeros(img_src.size(), img_src.type());
    Mat img_dst = Mat::zeros(img_src.size(), img_src.type());
    vector<Brick> bricks;
    cvtColor(img_src, img_hsv, COLOR_BGR2HSV); //RGB to HSV
    HsvRedColorSeg(img_hsv, img_hsv_seg);
    double diagonal_length = RectDetection(img_hsv_seg, img_dst, bricks);
    cout << "average diagonal length(pixels): " << diagonal_length << endl;
    for (size_t i = 0; i < bricks.size(); i++)
    {
        bricks[i].Print();
    }

    vector<Vector2d> points;
    Vector2d center;
    double min_dist;
    double forbidden_zone_radius = 2.0;

    // Test 1
    // points.push_back(Vector2d(-5.0, 0.0));
    // points.push_back(Vector2d(5.0, 0.0));
    // points.push_back(Vector2d(0.0, -5.0);
    // points.push_back(Vector2d(0.0, 5.0));

    // Test 2
    points.push_back(Vector2d(-1.5, 0.0));
    points.push_back(Vector2d(1.5, 0.0));
    points.push_back(Vector2d(0.0, -1.5));
    points.push_back(Vector2d(0.0, 1.5));

    geometricMedian(points, center, min_dist, 0.01, forbidden_zone_radius);

    cout << "coordinates of the geometric median point: " << endl;
    cout << center << endl;
    cout << "total distance: " << min_dist << endl;
    // imwrite(kFileDirPath+"img_src.jpg", img_src);
    // imwrite(kFileDirPath+"img_dst.jpg", img_dst);
    // waitKey(50);
    // system("pause");
}