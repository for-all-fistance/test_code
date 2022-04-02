#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

// // hmin, smin, vmin hmax, smax, vmax
// int hmin = 0, smin = 0, vmin = 12;
// int hmax = 38, smax = 255, vmax = 255;
// Scalar lower(hmin, smin, vmin);
// Scalar upper(hmax, smax, vmax);

const Mat cameraMatrix = (Mat_<double>(3, 3) <<  1.2853517927598091e+03, 0., 3.1944768628958542e+02, 0.,
       1.2792339468697937e+03, 2.3929354061292258e+02, 0., 0., 1.);//外参
const Mat distCoeff = (Mat_<double>(1, 5) <<  -6.3687295852461456e-01, -1.9748008790347320e+00,
       3.0970703651800782e-02, 2.1944646842516919e-03, 0. );//内参：畸变矩阵


int ImgPreProcess(Mat &img,Mat &dst,Size &imageSize,int mode)
{

       Mat map1, map2,calibratedImg;
       initUndistortRectifyMap(cameraMatrix, distCoeff, noArray(), cameraMatrix, imageSize, CV_32FC1, map1, map2);//对图像进行消歧
       remap(img, calibratedImg, map1, map2, INTER_NEAREST);//对每一帧图像进行消歧
       vector<Mat> channels;
       split(calibratedImg,channels);
       Mat Blur,threashould;
       GaussianBlur(channels[2],Blur,Size(7,7),0,0);//高斯滤波降噪
       //adaptiveThreshold(Blur,threashould,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,9,0);//自适应阈值化
       threshold(Blur,dst,0,255,THRESH_OTSU);
       // imshow("threashould",threashould);
       // Mat imgHSV,dst_HSV;
       // cvtColor(calibratedImg, imgHSV, COLOR_BGR2HSV);//转到HSV颜色空间
       // imshow("HSV",imgHSV);
       // inRange(imgHSV, lower, upper, dst_HSV);//图像二值化
       // imshow("InRange",dst_HSV);
       // bitwise_and(threashould,dst_HSV,dst);
       imshow("preproccesed",dst);
       return 0;
}


