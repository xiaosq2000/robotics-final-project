#include "rectangle-detection.h"

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

/**
 * @brief 
 * 
 * @param src - the preprocessed image, src.type() == 0
 * @param dst - the image to display
 * @param bricks - the std::vector store all detected bricks
 * @return double - the average length of diagonal lines of all bricks 
 */

void RectDetection(Mat &src, Mat &dst, vector<Brick> &bricks, double &diagonal_length)
{
    vector<vector<Point>> contours;
    findContours(src, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> rect(contours.size());
    Point2f vertices[4];
    
    for (size_t i = 0; i < contours.size(); i++)
    {
        rect[i] = minAreaRect(contours[i]);
        rect[i].points(vertices);
        for (size_t j = 0; j < 4; j++)
        {
            line(dst, vertices[j], vertices[(j + 1) % 4], Scalar(34, 34, 178), 2);
        }
        // the average of two diagonal lines for each brick
        diagonal_length += (norm(Mat(vertices[0]), Mat(vertices[2])) + norm(Mat(vertices[1]), Mat(vertices[3]))) / 2;
        bricks.push_back(Brick(Vector2d(double(rect[i].center.x), double(rect[i].center.y)), double(rect[i].angle)));
    }
    diagonal_length /= contours.size();
}
