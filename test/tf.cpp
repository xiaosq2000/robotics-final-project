/**
 * @file tf.cpp
 * @author 肖书奇
 * @brief HL机器人"坐标变换类"的测试
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#include "tf.h"

int main()
{
    // Construct a new `TF` object

    TF tf; // Use default parameters

    // Inverse kinematics test

    double rpy_coor[6]{495, -144, 530, -155, 178, -17};
    double joint_coor[6]{0};
    tf.World2Joint(rpy_coor, joint_coor);
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << joint_coor[i] << ",";
    }
    std::cout << std::endl;

    // Forward kinematics test

    // double joint_coor[6] {-16.0064, 22.887, 107.508, -1.68549, 51.1266, -56.9311};
    // double rpy_coor[6] {0};
    // tf.Joint2World(joint_coor, rpy_coor);
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout << rpy_coor[i] << ",";
    // }
    // std::cout << std::endl;

    // Pixel frame -> camera frame -> tool frame test

    double pixel_coor[2]{400, 400};
    double camera_coor[3]{0};
    double tool_coor[3]{0};
    tf.Pixel2Camera(pixel_coor, camera_coor);
    for (size_t i = 0; i < 3; i++)
    {
        std::cout << pixel_coor[i] << ",";
    }
    std::cout << std::endl;
    tf.Camera2Tool(camera_coor, tool_coor);
    for (size_t i = 0; i < 3; i++)
    {
        std::cout << tool_coor[i] << ",";
    }
    std::cout << std::endl;

    // Destory the object
    tf.~TF();
    return 0;
}