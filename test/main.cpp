
#include <iostream>
#include <string>
#include <filesystem>

#include "sample.h"
#include "FtpControl.h"
#include "communication.h"
#include "brick-construction.h"
#include "eye-in-hand-calibration.h"

int main()
{
    // Sampling for eye-in-hand calibration
    Sample calibration_sample("../share/eye-in-hand-calibration/src",
                              "../share/eye-in-hand-calibration/src/rpy.txt");
    calibration_sample.~Sample();

    // Calibration
    EyeInHandCalibration calibration("../share/eye-in-hand-calibration/src", 11, 8, 7, 7);
    calibration.Calibrate("../share/eye-in-hand-calibration/src");
    calibration.~EyeInHandCalibration();

    // Sampling for construction
    Sample src_sample("../share/sample",
                      "../share/sample/rpy.txt");
    src_sample.~Sample();

    // Detection, Generating trajectories, Log
    Construction construction;
    construction.BricksDetection();
    construction.Solution();
    construction.Log();

    // Ping-Pong buffer application
    Communication ppb_application;
    ppb_application.SendInstruction("[0# System.Auto 1]");
    ppb_application.SendInstruction("[0# PPB.Enable 1,1]");
    ppb_application.SendInstruction("[0# Robot.Frame 1,1]"); // joint space

    int traj_num = 0; // the number of trajectories
    for (const auto &entry : std::filesystem::directory_iterator("../share/motion-plan/dst"))
    {
        if (entry.path().extension() == ".txt")
        {
            std::cout << entry.path() << std::endl;
            FtpControl::Upload("192.168.10.101", "data", entry.path().string(), std::to_string(traj_num) + ".txt");
        }
        traj_num++;
    }
    for (size_t i = 0; i < traj_num; i++)
    {
        ppb_application.SendInstruction("[1# PPB.ReadFile 1,/data/" + std::to_string(i) + ".txt]");
        // 1 - move to the first point in the file, 0 - right-hand frame, 1 - rpy representation
        ppb_application.SendInstruction("[2# PPB.J2StartPoint 1,0,1]"); 
        ppb_application.SendInstruction("[3# PPB.Run 1]");
        ppb_application.SendInstruction("[4# WaitTime 2000]");
    }

    return 0;
}
