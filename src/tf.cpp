
/**
 * TODO: Refractor the relationship with class `EyeInHandCalibration`.
 *       especially, the `Pixel2Camera()` constant number 15 and path configuration.
 * 
 */

/**
 * @file tf.cpp
 * @author 肖书奇
 * @brief HL机器人"坐标变换类"的实现
 * 
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#include "tf.h"
#include <sstream>

const double PI = 3.1415926;

/**
 * @brief Construct a new TF::TF object
 * 
 * @param cam_param_directory 
 * @param src_directory 
 * @param target_height_offset 
 * @param l1 - link 1 length 
 * @param l2 - link 2 length 
 * @param l3 - link 3 length 
 * @param l4 - link 4 length 
 * 
 */
TF::TF(const std::string &cam_param_directory,
       const std::string &src_directory,
       const double &target_height_offset,
       const double &l1,
       const double &l2,
       const double &l3,
       const double &l4)
{
    cv::Mat cam_int_param, cam_ext_param;
    cv::FileStorage fs1(cam_param_directory + "/camera_intrinsic_matrix.yml", cv::FileStorage::READ);
    fs1["camera_intrinsic_matrix"] >> cam_int_param;
    fs1.release();
    cv::cv2eigen(cam_int_param, this->cam_int_param_);

    cv::FileStorage fs2(cam_param_directory + "/camera_extrinsic_matrix.yml", cv::FileStorage::READ);
    fs2["camera_extrinsic_matrix"] >> cam_ext_param;
    fs2.release();
    cv::cv2eigen(cam_ext_param, this->cam_ext_param_);

    this->link_length_1_ = l1;
    this->link_length_2_ = l2;
    this->link_length_3_ = l3;
    this->link_length_4_ = l4;

    this->w[0] = Vector3d(0, 0, 1);
    this->w[1] = Vector3d(0, 1, 0);
    this->w[2] = Vector3d(0, 1, 0);
    this->w[3] = Vector3d(0, 0, 1);
    this->w[4] = Vector3d(0, 1, 0);
    this->w[5] = Vector3d(0, 0, 1);

    this->q_[0] = Vector3d(0, 0, 0);
    this->q_[1] = Vector3d(0, 0, l1);
    this->q_[2] = Vector3d(0, 0, l1 + l2);
    this->q_[3] = Vector3d(0, 0, 0);
    this->q_[4] = Vector3d(0, 0, l1 + l2 + l3);
    this->q_[5] = Vector3d(0, 0, 0);

    // depth_
    this->target_height_offset_ = target_height_offset;
    this->depth_ = 0;
    std::ifstream rpy_file;
    rpy_file.open(src_directory + "/rpy.txt");
    std::string raw_str;
    std::getline(rpy_file, raw_str);
    double rpy_tool_src_config[6];
    std::stringstream input(raw_str); // segment
    std::string segmented_str;
    size_t i = 0;
    while (input >> segmented_str)
    {
        rpy_tool_src_config[i] = std::stod(segmented_str);
        i++;
    }
    this->depth_ += rpy_tool_src_config[2];
    this->depth_ -= target_height_offset;
}

/**
 * @brief Destroy the Rigid Body Transformation:: Rigid Body Transformation object
 * 
 * 
 */
TF::~TF()
{
}

/**
 * @brief Inverse Kinematics of HL-Robot.
 * 
 * @param g_vec - (INPUT) the 1D array (double [16]) containing the homogeneous matrix of the end-effector in world frame
 * @param joint_coor - (OUTPUT) the 1D array (double [6]) containing joint angles
 * @param config - the configuration of HL-Robot.
 * 
 */
void TF::InverseKinematics(const double *g_vec, double *joint_coor, bool *config)
{
    Vector3d v[6];
    for (int i = 0; i < 6; i++)
        v[i] = -w[i].cross(q_[i]);
    Matrix4d gst;
    for (int i = 0; i < 16; i++)
        gst(i / 4, i % 4) = g_vec[i];
    Matrix4d gst0 = Matrix4d::Zero();
    gst0.block<3, 3>(0, 0) << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    gst0.block<3, 1>(0, 3) = Vector3d(0, 0, this->link_length_1_ + this->link_length_2_ + this->link_length_3_ + this->link_length_4_);
    gst0(3, 3) = 1;
    Matrix4d g1 = gst * gst0.inverse();
    Vector4d p1(0, 0, this->link_length_1_ + this->link_length_2_ + this->link_length_3_, 1);
    Vector4d p2(0, 0, this->link_length_1_, 1);
    Vector3d r(0, 0, this->link_length_1_ + this->link_length_2_);
    PadenKahanSubproblem3(p1.block<3, 1>(0, 0), p2.block<3, 1>(0, 0), r, w[2], (g1 * p1 - p2).norm(), joint_coor[2]);
    Vector4d p3 = Twist2Homo(v[2], w[2], joint_coor[2]) * p1;
    Vector4d p4 = g1 * p1;
    r = Vector3d(0, 0, this->link_length_1_);
    PadenKahanSubproblem2(p3.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), r, w[0], w[1], joint_coor[0], joint_coor[1]);
    Vector4d p5(0, 0, this->link_length_1_ + this->link_length_2_ + this->link_length_3_ + this->link_length_4_, 1);
    Vector4d p6 = Twist2Homo(v[2], w[2], -joint_coor[2]) * Twist2Homo(v[1], w[1], -joint_coor[1]) * Twist2Homo(v[0], w[0], -joint_coor[0]) * g1 * p5;
    r = Vector3d(0, 0, this->link_length_1_ + this->link_length_2_ + this->link_length_3_);
    PadenKahanSubproblem2(p5.block<3, 1>(0, 0), p6.block<3, 1>(0, 0), r, w[3], w[4], joint_coor[3], joint_coor[4]);
    Vector4d p7(0, 1, 0, 0);
    r = Vector3d(0, 0, 0);
    Vector4d p8 = Twist2Homo(v[4], w[4], -joint_coor[4]) * Twist2Homo(v[3], w[3], -joint_coor[3]) * Twist2Homo(v[2], w[2], -joint_coor[2]) * Twist2Homo(v[1], w[1], -joint_coor[1]) * Twist2Homo(v[0], w[0], -joint_coor[0]) * g1 * p7;
    PadenKahanSubproblem1(p7.block<3, 1>(0, 0), p8.block<3, 1>(0, 0), r, w[5], joint_coor[5]);
}

/**
 * @brief Convert RPY coordinates(mm, deg) to joint coordinates(deg)
 * 
 * @param rpy_coor - (INPUT) RPY coordinates(mm, degree) of the end-effector
 * @param joint_coor - (OUTPUT) joint angles(deg)
 * 
 */
void TF::World2Joint(const double *rpy_coor, double *joint_coor)
{
    Matrix4d g = Rpy2Homo(rpy_coor[0], rpy_coor[1], rpy_coor[2], rpy_coor[3], rpy_coor[4], rpy_coor[5]);
    double g_vec[16];
    for (int i = 0; i < 16; i++)
    {
        g_vec[i] = g(i / 4, i % 4);
    }
    InverseKinematics(g_vec, joint_coor, this->config_);
    for (size_t i = 0; i < 6; i++)
    {
        joint_coor[i] *= (180 / PI);
    }
}

/**
 * @brief Forward Kinematics of HL-Robot
 * 
 * @param joint_coor - (INPUT) the 1D array (double [6]) containing joint angles
 * @param g_vec - (OUTPUT) the 1D array (double [16]) containing the homogeneous matrix of the end-effector in world frame
 * @param config - the configuration of HL-Robot.
 * 
 */
void TF::ForwardKinematics(const double *joint_coor, double *g_vec, bool *config)
{
    Vector3d v[6];
    for (int i = 0; i < 6; i++)
        v[i] = -w[i].cross(q_[i]);
    Matrix4d gst0 = Matrix4d::Zero();
    gst0.block<3, 3>(0, 0) << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    gst0.block<3, 1>(0, 3) = Vector3d(0, 0, this->link_length_1_ + this->link_length_2_ + this->link_length_3_ + this->link_length_4_);
    gst0(3, 3) = 1;
    Matrix4d g[6] = {Matrix4d::Zero()};
    for (int i = 0; i < 6; i++)
    {
        g[i] = Twist2Homo(v[i], w[i], joint_coor[i]);
    }
    Matrix4d gst = g[0];
    for (int i = 1; i < 6; i++)
        gst *= g[i];
    gst *= gst0;
    for (int i = 0; i < 16; i++)
        g_vec[i] = gst(i / 4, i % 4);
}

/**
 * @brief Convert joint coordinates(deg) to RPY coordinates(mm, deg)
 * 
 * @param joint_coor - (INPUT) joint angles(deg)
 * @param rpy_coor - (OUTPUT) RPY coordinates(mm, degree) of the end-effector
 * 
 */
void TF::Joint2World(const double *joint_coor, double *rpy_coor)
{
    double joint_coor_radian[6]{
        joint_coor[0] / 180 * PI,
        joint_coor[1] / 180 * PI,
        joint_coor[2] / 180 * PI,
        joint_coor[3] / 180 * PI,
        joint_coor[4] / 180 * PI,
        joint_coor[5] / 180 * PI};
    double Transmatrix[16];
    ForwardKinematics(joint_coor_radian, Transmatrix, this->config_);
    Matrix4d T;
    for (int i = 0; i < 16; i++)
    {
        T(i / 4, i % 4) = Transmatrix[i];
    }
    rpy_coor[0] = T(0, 3);
    rpy_coor[1] = T(1, 3);
    rpy_coor[2] = T(2, 3);
    Matrix4d T_r = T;
    T_r.block<3, 1>(0, 3) = Vector3d(0, 0, 0);
    Matrix4d g0 = Matrix4d::Zero();
    g0(0, 0) = g0(1, 1) = g0(2, 2) = g0(3, 3) = 1;
    Matrix4d g1 = T_r * g0.inverse();
    Vector4d p1 = Vector4d(0, 0, 1, 1);
    Vector4d p2 = g1 * p1;
    Vector3d r = Vector3d(0, 0, 0);
    PadenKahanSubproblem2(p1.block<3, 1>(0, 0), p2.block<3, 1>(0, 0), r, Vector3d(0, 0, 1), Vector3d(0, 1, 0), rpy_coor[3], rpy_coor[4]);
    Vector4d p3 = Vector4d(0, 1, 0, 1);
    Vector4d p4 = Twist2Homo(Vector3d(0, 0, 0), Vector3d(0, 1, 0), -rpy_coor[4]) * Twist2Homo(Vector3d(0, 0, 0), Vector3d(0, 0, 1), -rpy_coor[3]) * g1 * p3;
    PadenKahanSubproblem1(p3.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), r, Vector3d(0, 0, 1), rpy_coor[5]);
    //单位转换，将xyz转换为mm，ypr转换为度
    rpy_coor[0] *= 1000;
    rpy_coor[1] *= 1000;
    rpy_coor[2] *= 1000;
    rpy_coor[3] *= 180 / PI;
    rpy_coor[4] *= 180 / PI;
    rpy_coor[5] *= 180 / PI;
}

/**
 * @brief Convert twist representation to homogeneous matrix representation
 * 
 * @param v - the translation vector
 * @param w - the equivalent axis
 * @param theta - rotation angle
 * 
 * @return Matrix4d - homogeneous matrix representation
 */
Matrix4d Twist2Homo(Vector3d v, Vector3d w, double theta)
{
    Matrix3d R, w_hat;
    Vector3d p;
    Matrix4d g = Matrix4d::Zero();
    w_hat << 0, -w(2), w(1),
        w(2), 0, -w(0),
        -w(1), w(0), 0;
    R = Matrix3d::Identity() + w_hat * sin(theta) + w_hat * w_hat * (1 - cos(theta));
    p = (Matrix3d::Identity() - R) * w.cross(v) + w * w.dot(v * theta);
    g.block<3, 3>(0, 0) = R;
    g.block<3, 1>(0, 3) = p;
    g(3, 3) = 1;
    return g;
}

/**
 * @brief Convert RPY representation to homogeneous matrix representation
 * 
 * @param x - (mm)
 * @param y - (mm) 
 * @param z - (mm) 
 * @param yaw - (deg) 
 * @param pitch - (deg)  
 * @param roll - (deg)  
 * 
 * @return Matrix4d - homogeneous matrix representation(m, rad)  
 */
Matrix4d Rpy2Homo(double x, double y, double z, double yaw, double pitch, double roll)
{
    x /= 1000.0;
    y /= 1000.0;
    z /= 1000.0;
    yaw /= 180 / PI;
    pitch /= 180 / PI;
    roll /= 180 / PI;

    Matrix4d g0 = Matrix4d::Zero();
    g0(0, 0) = g0(1, 1) = g0(2, 2) = g0(3, 3) = 1;
    Matrix4d g1 = Twist2Homo(Vector3d(0, 0, 0), Vector3d(0, 0, 1), yaw);
    Matrix4d g2 = Twist2Homo(Vector3d(0, 0, 0), Vector3d(0, 1, 0), pitch);
    Matrix4d g3 = Twist2Homo(Vector3d(0, 0, 0), Vector3d(0, 0, 1), roll);
    Matrix4d g_r = g1 * g2 * g3 * g0;
    Matrix4d g_end = Matrix4d::Zero();
    g_end.block<3, 3>(0, 0) = g_r.block<3, 3>(0, 0);
    g_end.block<3, 1>(0, 3) = Vector3d(x, y, z);
    g_end(3, 3) = 1;
    return g_end;
}

/**
 * @brief Paden-Kahan subproblem 1
 * 
 * @param p - (m)
 * @param q - (m) 
 * @param r - (m) 
 * @param w - (m) 
 * @param theta - (rad)
 * 
 */
void PadenKahanSubproblem1(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double &theta)
{
    Vector3d u = p - r, v = q - r;
    Vector3d up = u - w * w.dot(u);
    Vector3d vp = v - w * w.dot(v);
    if (fabs(w.dot(u) - w.dot(v)) > 1e-6 || fabs(up.norm() - vp.norm()) > 1e-6)
    {
        std::cout << "No solution for subproblem1! Because w'u /= w'v or ||u'||/=||v'||" << std::endl;
        std::cout << "w'u=" << w.dot(u) << "\t"
                  << "w'v=" << w.dot(v) << std::endl;
        std::cout << "||u'||=" << up.norm() << "\t||v'||=" << vp.norm() << std::endl;
        theta = 0;
        return;
    }
    if (up.norm() > 1e-6)
    {
        theta = atan2(w.dot(up.cross(vp)), up.dot(vp));
    }
    else
    {
        std::cout << "No solution for subproblem1! Because ||u'||=0" << std::endl;
        theta = 0;
    }
}

/**
 * @brief Paden-Kahan subproblem 2
 * 
 * @param p - (m)
 * @param q - (m)
 * @param r - (m)
 * @param w1 - (m)
 * @param w2 - (m)
 * @param theta1 - (rad)
 * @param theta2 - (rad)
 * 
 */
void PadenKahanSubproblem2(Vector3d p, Vector3d q, Vector3d r, Vector3d w1, Vector3d w2, double &theta1, double &theta2)
{
    Vector3d u = p - r, v = q - r;
    if (fabs(u.norm() - v.norm()) > 1e-6)
    {
        std::cout << "No solution for subproblem2! Because ||u||/=||v||" << std::endl;
        std::cout << "||u||=" << u.norm() << "\t||v||=" << v.norm() << std::endl;
        theta1 = theta2;
        return;
    }
    double alpha = ((w1.dot(w2)) * w2.dot(u) - w1.dot(v)) / (pow(w1.dot(w2), 2) - 1);
    double beta = ((w1.dot(w2)) * w1.dot(v) - w2.dot(u)) / (pow(w1.dot(w2), 2) - 1);
    double gamma = -sqrt((pow(u.norm(), 2) - pow(alpha, 2) - pow(beta, 2) - 2 * alpha * beta * w1.dot(w2)) / pow(w1.cross(w2).norm(), 2));
    Vector3d z = alpha * w1 + beta * w2 + gamma * w1.cross(w2);
    Vector3d c = z + r;
    PadenKahanSubproblem1(p, c, r, w2, theta2);
    PadenKahanSubproblem1(q, c, r, -w1, theta1);
}

/**
 * @brief Paden-Kahan subproblem 3
 * 
 * @param p - (m)
 * @param q - (m)
 * @param r - (m) 
 * @param w - (m) 
 * @param d - (m) 
 * @param theta - (rad) 
 * 
 */
void PadenKahanSubproblem3(Vector3d p, Vector3d q, Vector3d r, Vector3d w, double d, double &theta)
{
    Vector3d u = p - r, v = q - r;
    Vector3d up = u - w * w.dot(u);
    Vector3d vp = v - w * w.dot(v);
    if (fabs(w.dot(u) - w.dot(v)) > 1e-6 || fabs(up.norm() - vp.norm()) > 1e-6)
    {
        std::cout << "No solution for subproblem1! Because w' /= tw'v or ||u'||/=||v'||" << std::endl;
        std::cout << "w'u=" << w.dot(u) << "\t"
                  << "w'v=" << w.dot(v) << std::endl;
        std::cout << "||u'||=" << up.norm() << "\t||v'||=" << vp.norm() << std::endl;
        theta = 0;
        return;
    }
    double theta0 = atan2(w.dot(up.cross(vp)), up.dot(vp));
    double dp2 = pow(d, 2) - pow(w.dot(p - q), 2);
    theta = theta0 - acos((pow(up.norm(), 2) + pow(vp.norm(), 2) - dp2) / (2 * up.norm() * vp.norm()));
}

/**
 * @brief Convert pixel information to coordinates in camera frame
 * 
 * @param pixel_coor - (INPUT) double [2]
 * @param camera_coor - (OUTPUT) double [3] 
 * 
 */
void TF::Pixel2Camera(const double *pixel_coor, double *camera_coor)
{
    std::ifstream offset_file;
    offset_file.open("../share/eye-in-hand-calibration/dst/offset.txt", std::ios::in);
    std::string raw_str;
    std::getline(offset_file, raw_str);
    this->depth_ = 100 + (std::stod(raw_str) - this->target_height_offset_);
    offset_file.close();

    Vector3d pixel_homo_coor{
        pixel_coor[0],
        pixel_coor[1],
        1};
    Vector3d camera_homo_coor = this->cam_int_param_.inverse() * this->depth_ * pixel_homo_coor;
    camera_coor[0] = camera_homo_coor[0];
    camera_coor[1] = camera_homo_coor[1];
    camera_coor[2] = camera_homo_coor[2];
}

/**
 * @brief Convert coordinates in camera frame to coordinates in tool frame
 * 
 * @param camera_coor - (INPUT) double [3]
 * @param tool_coor - (OUTPUT) double [3]  
 * 
 */
void TF::Camera2Tool(const double *camera_coor, double *tool_coor)
{
    Vector4d camera_homo_coor{
        camera_coor[0],
        camera_coor[1],
        camera_coor[2],
        1};
    Vector4d tool_homo_coor = this->cam_ext_param_ * camera_homo_coor;
    tool_coor[0] = tool_homo_coor[0];
    tool_coor[1] = tool_homo_coor[1];
    tool_coor[2] = tool_homo_coor[2];
}

void TF::Tool2World(const double *rpy_tool_config, const double *tool_coor, double *rpy_coor)
{
    Matrix4d g_world2tool = Matrix4d::Zero();
    g_world2tool = Rpy2Homo(rpy_tool_config[0], rpy_tool_config[1], rpy_tool_config[2], rpy_tool_config[3], rpy_tool_config[4], rpy_tool_config[5]);
    Vector4d tmp_tool_coor{tool_coor[0], tool_coor[1], tool_coor[2], 1};
    Vector4d tmp_rpy_coor = Vector4d::Zero();
    tmp_rpy_coor = g_world2tool * tmp_tool_coor;
    for (size_t i = 0; i < 3; i++)
    {
        rpy_coor[i] = tmp_rpy_coor[i];
    }
}
