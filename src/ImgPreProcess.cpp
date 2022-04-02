#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

// hmin, smin, vmin hmax, smax, vmax
vector<int> armerColors{0, 0, 0, 0, 0, 0} ;//目标色彩值
vector<int> energyColors{0, 0, 0, 0, 0, 0};

const Mat cameraMatrix = (Mat_<double>(3, 3) <<  1.2853517927598091e+03, 0., 3.1944768628958542e+02, 0.,
       1.2792339468697937e+03, 2.3929354061292258e+02, 0., 0., 1.);//外参
const Mat distCoeff = (Mat_<double>(1, 5) <<  -6.3687295852461456e-01, -1.9748008790347320e+00,
       3.0970703651800782e-02, 2.1944646842516919e-03, 0. );//内参：畸变矩阵


int ImgPreProcess(Mat &img,Mat &dst,Size &imageSize,int mode)
{
   Scalar lower,upper;
   Mat map1, map2,calibratedImg;
   initUndistortRectifyMap(cameraMatrix, distCoeff, noArray(), cameraMatrix, imageSize, CV_32FC1, map1, map2);//对图像进行消歧
   remap(img, calibratedImg, map1, map2, INTER_NEAREST);//对每一帧图像进行消歧
   vector<Mat> channels;
   split(calibratedImg,channels);
   Mat Blur;
   // if(mode==0)
   // cvtColor(calibratedImg,channels,COLOR_BGR2GRAY);
   // imshow("gray",channels);
   GaussianBlur(channels[2],Blur,Size(7,7),0,0);//高斯滤波降噪
   // Mat element =getStructuringElement(MORPH_RECT,Size(6,6));//获取自定义核
   // morphologyEx(channels[2],Blur,MORPH_OPEN,element);
   adaptiveThreshold(Blur,dst,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,3,1);//自适应阈值化

   imshow("gaussian_threshould",dst);
   // Mat element=getStructuringElement(MORPH_RECT,Size(18,18));//get an rect kernel of size(18,18)
   // Mat test1,test2;
   // morphologyEx(img,test1,MORPH_CLOSE,element);
   // morphologyEx(img,test2,MORPH_OPEN,element);
   // imshow("OPEN",test2);
   // imshow("close",test1);

   // Mat imgHSV;
   // cvtColor(calibratedImg, imgHSV, COLOR_BGR2HSV);//转到HSV颜色空间
   // inRange(imgHSV, lower, upper, dst);//图像二值化
   return 0;
}


