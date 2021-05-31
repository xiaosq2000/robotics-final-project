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
    double rpy_coor_begin[6]{415.746, -219.622, 920.636,
                             -29.489, 178.867, -33.751};
    double rpy_coor_end[6]{374.286, 208.621, 884.491,
                           -8.287, 178.867, -33.75};
    motion_plan.Config(rpy_coor_begin, rpy_coor_end, 0.005, 100, 100, -100);
    motion_plan.GenerateJointPointsFile("../share/motion-plan/test_1.txt");
    motion_plan.Simulate("../share/motion-plan/simulation_1.txt");
    motion_plan.~MotionPlan();
    return 0;
}
