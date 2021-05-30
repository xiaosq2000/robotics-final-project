
#ifndef _EYE_IN_HAND_CALIBRATION_H_
#define _EYE_IN_HAND_CALIBRATION_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

void RR_2R(cv::Mat &RR, cv::Mat &TT, cv::Mat &R, cv::Mat &T, int i);
cv::Mat RT2H(cv::Mat &R, cv::Mat &T);
void H2RT(cv::Mat &H, cv::Mat &R, cv::Mat &T);
bool isRotationMatrix(const cv::Mat &R);
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat &eulerAngle, const std::string &seq);
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d &q);
cv::Mat attitudeVectorToMatrix(const cv::Mat &m, bool useQuaternion, const std::string &seq);

class EyeInHandCalibration
{
private:
    std::vector<cv::Mat> imgs_src_;
    std::vector<cv::Mat> poses_src_;
    cv::Size board_size_;
    cv::Size square_size_;

    cv::Mat cam_int_param_;
    cv::Mat cam_dist_param_;
    cv::Mat cam_ext_param_;

public:
    EyeInHandCalibration(std::string path, int board_width, int board_height, int square_width, int square_height);
    ~EyeInHandCalibration();
    void Calibrate(std::string path);
    void Save(std::string path);
};

#endif
