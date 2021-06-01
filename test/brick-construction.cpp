
/**
 * @file brick-construction.cpp
 * @author 肖书奇
 * @brief “积木类”与“搭建类”的测试
 * @version 1.0
 * @date 2021-06-01
 * 
 */
#include "brick-construction.h"

int main()
{
    double rpy_tool_src[6]{489.007, 16.974, 582.742, 0.000, 180.000, 40.000};
    Construction construction(rpy_tool_src);
    construction.BricksDetection();
    construction.Solution();
    construction.Log();
    return 0;
}
