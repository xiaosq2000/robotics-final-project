
/**
 * @file motion-plan.cpp
 * @author 肖书奇
 * @brief HL机器人“运动规划类”的测试
 * @version 1.0
 * @date 2021-05-31
 * 
 */

#include "motion-plan.h"

int main()
{
    MotionPlan motion_plan;
    double rpy_coor_begin[6]{415.746, -219.622, 920.636, -29.489, 178.867, -33.751};
    double rpy_coor_end[6]{555.893, -54.405, 920.636, -8.288, 178.867, -33.751};
    motion_plan.Config(rpy_coor_begin, rpy_coor_end, 0.002, 10, 10, 10, "line", "trapezoid");
    motion_plan.GenerateJointPointsFile("../share/motion-plan/test.txt");
    motion_plan.Simulate("../share/motion-plan/simulation/test_simulation.txt");
    motion_plan.~MotionPlan();
    return 0;
}
