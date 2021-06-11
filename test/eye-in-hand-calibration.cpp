
/**
 * @file eye-in-hand-calibration.cpp
 * @author 肖书奇
 * @brief “手眼标定类”的测试
 * @version 1.0
 * @date 2021-05-30
 * 
 */

#include "eye-in-hand-calibration.h"

int main()
{
    EyeInHandCalibration eye_in_hand_calibration("../share/eye-in-hand-calibration/src", 11, 8, 7, 7);
    eye_in_hand_calibration.Calibrate("../share/eye-in-hand-calibration/dst");
    eye_in_hand_calibration.~EyeInHandCalibration();
    // system("pause");
    return 0;
}
