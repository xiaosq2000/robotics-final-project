
#include <iostream>
#include <string>
#include <fstream>
#include <winsock.h>
#include <conio.h>

#include "rs.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "communication.h"
#include "sample.h"

/**
 * @brief Construct a new Sample:: Sample object
 * 
 * @param img_directory: the directory to store images
 * @param rpy_file_path: the path of the ".txt" file within rpy configurations of the tool, where images will be captured
 * 
 */
Sample::Sample(std::string img_directory, std::string rpy_file_path)
{
    this->img_directory_ = img_directory;
    this->rpy_file_path_ = rpy_file_path;
    std::ifstream rpy_file;
    rpy_file.open(this->rpy_file_path_, std::ios::in);
    if (!rpy_file)
    {
        std::cout << "File open error!" << std::endl;
    }

    Communication communication;

    size_t i = 1;
    std::string raw_str;
    std::string segmented_str;
    double rpy_tool_config[6];
    while (std::getline(rpy_file, raw_str))
    {

        // Move
        communication.SendInstruction("[1# System.Auto 1]");
        communication.SendInstruction("[2# System.Speed 10]");
        std::stringstream input(raw_str);
        size_t j = 0;
        std::cout.precision(3);
        while (input >> segmented_str)
        {
            rpy_tool_config[j] = std::stod(segmented_str);
            j++;
        }
        std::string rpy_tool_config_sep_comma{0};
        rpy_tool_config_sep_comma = std::to_string(rpy_tool_config[0]) + "," + std::to_string(rpy_tool_config[1]) + "," + std::to_string(rpy_tool_config[2]) + "," + std::to_string(rpy_tool_config[3]) + "," + std::to_string(rpy_tool_config[4]) + "," + std::to_string(rpy_tool_config[5]);
        std::cout << "[3# Location samplePose=" + rpy_tool_config_sep_comma + "]" << std::endl;
        communication.SendInstruction("[3# Location samplePose=" + rpy_tool_config_sep_comma + "]");
        communication.SendInstruction("[4# Move.Joint samplePose]");

        // Take a picture
        rs2::colorizer color_map;
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);
        rs2::frameset data = pipe.wait_for_frames();
        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth = data.get_depth_frame();
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
        if (i < 10)
        {
            cv::imwrite(this->img_directory_ + "/0" + std::to_string(i) + ".jpg", image);
        }
        else
        {
            cv::imwrite(this->img_directory_ + "/" + std::to_string(i) + ".jpg", image);
        }
        i++;
        // std::cout << raw_str << std::endl;
    }
}

Sample::~Sample()
{
}
