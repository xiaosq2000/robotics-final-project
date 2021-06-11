
/**
 * @file eye-in-hand-calibration.h
 * @author 肖书奇
 * @brief “手眼标定类”的声明
 * @version 1.0
 * @date 2021-06-04
 * 
 */

#ifndef _EYE_IN_HAND_CALIBRATION_H_
#define _EYE_IN_HAND_CALIBRATION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

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
    std::vector<cv::Mat> rpy_tool_config_;
    cv::Size board_size_;
    cv::Size square_size_;

    cv::Mat cam_int_param_;
    cv::Mat cam_dist_param_;
    cv::Mat cam_ext_param_;

    double offset_;

public:
    EyeInHandCalibration(std::string src_directory = "../share/eye-in-hand-calibration/src", int board_width = 11, int board_height = 8, int square_width = 7, int square_height = 7);
    ~EyeInHandCalibration();
    void Calibrate(std::string dst_directory);
};

#endif
