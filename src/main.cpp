
#include "brick.h"
#include "construction.h"
#include "rectangle-detection.h"
#include "HLrobotconfig.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const string kFileDirPath = "C:/toy-projects/robotics-final-project/robotics-final-project/data";

/**
 * @brief The entry of the test application.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

    // double joint[6] {0};
    // HLRobot::SetRobotEndPos(495, -144, 530, -155, 178, -17);
    // HLRobot::GetJointAngles(joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    // for (size_t i = 0; i < 6; i++)
    // {
    //     cout << joint[i] << ", " ;
    // }
    // cout << endl;

    Mat img_src = imread(kFileDirPath + "/rectangle-detection/img_src.jpg");
    Mat img_hsv = Mat::zeros(img_src.size(), img_src.type());
    Mat img_hsv_seg = Mat::zeros(img_src.size(), img_src.type());
    Mat img_dst = Mat::zeros(img_src.size(), img_src.type());

    vector<Brick> bricks;   // to store all detected bricks
    double diagonal_length; // to store average length of diagonal line of bricks (pixels)

    cvtColor(img_src, img_hsv, COLOR_BGR2HSV);                    // RGB to HSV
    HsvRedColorSeg(img_hsv, img_hsv_seg);                         // HSV segmentation
    RectDetection(img_hsv_seg, img_dst, bricks, diagonal_length); // rectangle detection

    Construction construction(bricks, diagonal_length);
    construction.FindBestLocationCamera();
    construction.Label(false);
    construction.Display(img_dst);
    construction.WriteToFile();
    construction.MotionPlan();

    imwrite(kFileDirPath + "/rectangle-detection/img_dst.jpg", img_dst);
    system("pause");

}
