
/**
 * @file eye-in-hand-calibration.cpp
 * @author 肖书奇
 * @brief “手眼标定类”的实现
 * @version 1.0
 * @date 2021-06-04
 * 
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "eye-in-hand-calibration.h"

/**
 * @brief Construct a new Eye In Hand Calibration:: Eye In Hand Calibration object
 * 
 * @param src_directory 
 * @param board_width 
 * @param board_height 
 * @param square_width 
 * @param square_height 
 * 
 */

EyeInHandCalibration::EyeInHandCalibration(std::string src_directory, int board_width, int board_height, int square_width, int square_height)
{
    std::cout << "Loading images..." << std::endl;
    for (const auto &entry : std::filesystem::directory_iterator(src_directory))
    {
        if (entry.path().extension() == ".png")
        {
            std::cout << entry.path() << std::endl;
            this->imgs_src_.push_back(cv::imread(entry.path().string()));
        }
        if (entry.path().extension() == ".txt")
        {
            std::cout << "Loading poses..." << std::endl;
            std::ifstream file_poses;
            file_poses.open(entry.path());

            std::string raw_str;
            size_t i = 0;
            while (std::getline(file_poses, raw_str))
            {

                std::cout << raw_str << std::endl;

                cv::Mat tmp_pose(1, 6, CV_64F);
                std::stringstream input(raw_str);
                std::string segmented_str;
                size_t j = 0;
                while (input >> segmented_str)
                {
                    if (i == 0 && j == 2)
                    {
                        this->offset_ = std::stod(segmented_str);
                    }
                    tmp_pose.at<double>(0, j) = std::stod(segmented_str);
                    j++;
                }
                this->rpy_tool_config_.push_back(tmp_pose);
                i++;
            }
            file_poses.close();
        }
    }

    std::cout << "Loading pattern information..." << std::endl;
    this->board_size_ = cv::Size(board_width, board_height);
    this->square_size_ = cv::Size(square_width, square_width);
    this->cam_int_param_ = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    this->cam_dist_param_ = cv::Mat(1, 5, CV_64FC1, cv::Scalar::all(0));
    std::cout << "board_size_" << this->board_size_ << std::endl;
    std::cout << "square_size_" << this->square_size_ << std::endl;
}

EyeInHandCalibration::~EyeInHandCalibration()
{
}

void EyeInHandCalibration::Calibrate(std::string dst_directory)
{
    std::cout << "Calibrating camera intrinsic parameters..." << std::endl;

    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<std::vector<cv::Point3f>> all_corners_coor;
    all_corners.reserve(this->imgs_src_.size());
    all_corners_coor.reserve(this->imgs_src_.size());

    for (size_t i = 0; i < this->imgs_src_.size(); i++)
    {
        cv::Mat img_dst;
        cv::Point3f corner_coor;
        std::vector<cv::Point3f> corners_coor;
        cv::findChessboardCorners(this->imgs_src_[i], this->board_size_, corners);
        cv::cvtColor(this->imgs_src_[i], img_dst, cv::COLOR_RGB2GRAY);
        cv::find4QuadCornerSubpix(img_dst, corners, cv::Size(5, 5));
        cv::drawChessboardCorners(img_dst, this->board_size_, corners, true);
        all_corners.push_back(corners);
        cv::imwrite(dst_directory + "/img_dst_" + std::to_string(i) + ".png", img_dst);

        for (int j = 0; j < this->board_size_.height; j++)
        {
            for (int k = 0; k < this->board_size_.width; k++)
            {
                corner_coor.x = k * this->square_size_.width;
                corner_coor.y = j * this->square_size_.height;
                corner_coor.z = 0;
                corners_coor.push_back(corner_coor);
            }
        }
        all_corners_coor.push_back(corners_coor);
    }

    std::vector<cv::Mat> cam_translation;
    std::vector<cv::Mat> cam_rotation;
    cv::calibrateCamera(all_corners_coor, all_corners, this->imgs_src_[1].size(), this->cam_int_param_, this->cam_dist_param_, cam_translation, cam_rotation, 0);

    std::cout << "camera intrinsic matrix" << std::endl
              << this->cam_int_param_ << std::endl
              << "camera distortion matrix" << std::endl
              << this->cam_dist_param_ << std::endl;

    std::cout << "Saving to " + dst_directory + "/camera_intrinsic_matrix.yml" << std::endl;
    cv::FileStorage fs1(dst_directory + "/camera_intrinsic_matrix.yml", cv::FileStorage::WRITE);
    fs1 << "camera_intrinsic_matrix" << this->cam_int_param_;
    fs1.release();

    std::cout << "Calibrating camera extrinsic parameters..." << std::endl;

    std::vector<cv::Mat> target2cam_rotation;
    std::vector<cv::Mat> target2cam_translation;
    std::vector<cv::Mat> target2cam_homogeneity;

    for (size_t i = 0; i < this->imgs_src_.size(); i++)
    {
        cv::Mat tmp_r, tmp_t, pnp_r, pnp_t, tmp_h;
        // solve PnP problem
        cv::solvePnP(all_corners_coor[i], all_corners[i], this->cam_int_param_, this->cam_dist_param_, pnp_r, pnp_t, false, cv::SOLVEPNP_ITERATIVE);
        // convert
        cv::Rodrigues(pnp_r, tmp_r);
        tmp_t = pnp_t / 1000.0f;
        tmp_h = RT2H(tmp_r, tmp_t);
        // store
        target2cam_rotation.push_back(tmp_r);
        target2cam_translation.push_back(tmp_t);
        target2cam_homogeneity.push_back(tmp_h);
    }

    std::vector<cv::Mat> gripper2base_rotation;
    std::vector<cv::Mat> gripper2base_translation;
    std::vector<cv::Mat> gripper2base_homogeneity;
    for (size_t i = 0; i < this->rpy_tool_config_.size(); i++)
    {
        cv::Mat tmp_r, tmp_t, tmp_h;
        tmp_h = attitudeVectorToMatrix(this->rpy_tool_config_[i], false, "zyz");
        H2RT(tmp_h, tmp_r, tmp_t);
        gripper2base_rotation.push_back(tmp_r);
        gripper2base_translation.push_back(tmp_t);
        gripper2base_homogeneity.push_back(tmp_h);
    }

    this->offset_ = this->offset_ - target2cam_translation[0].at<double>(2, 0) * 1000;
    std::cout << std::endl;
    std::cout << "Offset: " << std::endl;
    std::cout << this->offset_ << std::endl;
    std::cout << std::endl;

    std::cout << "Saving to" + dst_directory + "/offset.txt" << std::endl;
    std::ofstream offset_file;
    offset_file.open(dst_directory + "/offset.txt", std::ios::ate);
    if (!offset_file)
    {
        std::cout << "File open error!" << std::endl;
    }
    offset_file << this->offset_ << std::endl;
    offset_file.close();

    cv::Mat cam2gripper_rotation = (cv::Mat_<double>(3, 3));
    cv::Mat cam2gripper_translation = (cv::Mat_<double>(3, 1));
    cv::Mat cam2gripper_homogeneity;
    cv::calibrateHandEye(gripper2base_rotation, gripper2base_translation, target2cam_rotation, target2cam_translation, cam2gripper_rotation, cam2gripper_translation, cv::CALIB_HAND_EYE_TSAI);
    cam2gripper_homogeneity = RT2H(cam2gripper_rotation, cam2gripper_translation);
    this->cam_ext_param_ = cam2gripper_homogeneity.clone();

    std::cout << this->cam_ext_param_ << std::endl;

    std::cout << "Saving to " + dst_directory + "/camera_extrinsic_matrix.yml" << std::endl;
    cv::FileStorage fs2(dst_directory + "/camera_extrinsic_matrix.yml", cv::FileStorage::WRITE);
    fs2 << "camera_extrinsic_matrix" << this->cam_ext_param_;
    fs2.release();
}

/**
 * @brief Convert homogenenous matrix to rotation matrix and translation matrix
 * 
 * @param r: rotation matrix
 * @param t: tranlantion matrix 
 * 
 * @return cv::Mat homogenenous matrix
 */
cv::Mat RT2H(cv::Mat &r, cv::Mat &t)
{
    cv::Mat tmp_h;
    cv::Mat_<double> tmp_r = (cv::Mat_<double>(4, 3) << r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
                              r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2),
                              r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2),
                              0.0, 0.0, 0.0);
    cv::Mat_<double> tmp_t = (cv::Mat_<double>(4, 1) << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0), 1.0);
    cv::hconcat(tmp_r, tmp_t, tmp_h);
    return tmp_h;
}
/**
 * @brief Convert homogenenous matrix to rotation matrix and translation matrix
 * 
 * @param h: homogeneous matrix
 * @param r: rotation matrix 
 * @param t: translation matrix
 * 
 */
void H2RT(cv::Mat &h, cv::Mat &r, cv::Mat &t)
{
    cv::Rect r_rect(0, 0, 3, 3);
    cv::Rect t_rect(3, 0, 1, 3);
    r = h(r_rect);
    t = h(t_rect);
}
bool isRotationMatrix(const cv::Mat &R)
{
    cv::Mat tmp33 = R({0, 0, 3, 3});
    cv::Mat shouldBeIdentity;

    shouldBeIdentity = tmp33.t() * tmp33;

    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(I, shouldBeIdentity) < 1e-6;
}

cv::Mat eulerAngleToRotatedMatrix(const cv::Mat &eulerAngle, const std::string &seq)
{
    CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

    eulerAngle /= 180 / CV_PI;
    cv::Matx13d m(eulerAngle);
    auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
    auto xs = std::sin(rx), xc = std::cos(rx);
    auto ys = std::sin(ry), yc = std::cos(ry);
    auto zs = std::sin(rz), zc = std::cos(rz);

    cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
    cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
    cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);
    cv::Mat rotZ1 = (cv::Mat_<double>(3, 3) << xc, -xs, 0, xs, xc, 0, 0, 0, 1);
    cv::Mat rotMat;

    if (seq == "zyx")
        rotMat = rotX * rotY * rotZ;
    else if (seq == "yzx")
        rotMat = rotX * rotZ * rotY;
    else if (seq == "zxy")
        rotMat = rotY * rotX * rotZ;
    else if (seq == "xzy")
        rotMat = rotY * rotZ * rotX;
    else if (seq == "yxz")
        rotMat = rotZ * rotX * rotY;
    else if (seq == "xyz")
        rotMat = rotZ * rotY * rotX;
    else if (seq == "zyz")
        rotMat = rotZ1 * rotY * rotZ;
    else
    {
        cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
                  __FUNCTION__, __FILE__, __LINE__);
    }

    if (!isRotationMatrix(rotMat))
    {
        cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
                  __FUNCTION__, __FILE__, __LINE__);
    }

    return rotMat;
    //cout << isRotationMatrix(rotMat) << endl;
}
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d &q)
{
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double x2 = x * x, y2 = y * y, z2 = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    cv::Matx33d res{
        1 - 2 * (y2 + z2),
        2 * (xy - wz),
        2 * (xz + wy),
        2 * (xy + wz),
        1 - 2 * (x2 + z2),
        2 * (yz - wx),
        2 * (xz - wy),
        2 * (yz + wx),
        1 - 2 * (x2 + y2),
    };
    return cv::Mat(res);
}
cv::Mat attitudeVectorToMatrix(const cv::Mat &m, bool useQuaternion, const std::string &seq)
{
    CV_Assert(m.total() == 6 || m.total() == 10);
    /*if (m.cols == 1)
		m = m.t();*/
    cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

    //如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
    if (useQuaternion) // normalized vector, its norm should be 1.
    {
        cv::Vec4d quaternionVec = m({3, 0, 4, 1});
        quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({0, 0, 3, 3}));
        // cout << norm(quaternionVec) << endl;
    }
    else
    {
        cv::Mat rotVec;
        if (m.total() == 6)
            rotVec = m({3, 0, 3, 1}); //6
        else
            rotVec = m({7, 0, 3, 1}); //10

        //如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
        if (0 == seq.compare(""))
            cv::Rodrigues(rotVec, tmp({0, 0, 3, 3}));
        else
            eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({0, 0, 3, 3}));
    }
    tmp({3, 0, 1, 3}) = m({0, 0, 3, 1}).t() / 1000.0f;

    return tmp;
}
