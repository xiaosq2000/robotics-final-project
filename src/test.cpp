/* Includes ----------------------------------------*/
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
/* Namespace ----------------------------------------*/
using namespace std;
using namespace cv;
/* Gloabl constant variable ----------------------------------------*/

/**
 *  @brief{The entry of the application}
 *  @param{int argc}, @param{char** argv}
 *  @retval{int}
 */

int main( int argc, char** argv )
{   
    Mat a = Mat::zeros(Size(10,10),0);
    cout << a << endl;
}