#ifndef imageProcess_hpp_
#define imageProcess_hpp_

//-----------------------------------【头文件包含部分】------------------------------ 
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include <single_led/vlcCommonInclude.hpp>

//-----------------------------------【命名空间声明分】------------------------------ 
//      描述：包含程序所使用的命名空间  
//----------------------------------------------------------------------------------------------  
using namespace cv;

//-----------------------------------【图像处理函数声明】---------------------------
double getThreshVal_Otsu_8u(const cv::Mat& _src);
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next);
void LED_contour(const Mat& _img, vector<Point> &myPoints, vector<int> &myRadius);
void para_set(double& X, double& Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H);
double get_similarity(Mat&a,Mat&b);
Rect find_minrect(Mat frame,Rect selected,int thresh);
Rect ls_LED_sample(Mat img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext,int sample);
void thinImage(Mat &srcimage);
#endif
