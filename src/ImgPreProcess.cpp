#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "/home/pi/Desktop/xbotcon/include/DEBUG.h"
#include "/home/pi/Desktop/xbotcon/include/head.h"
using namespace cv;
using namespace std;

int ImgPreProcess_DEBUG(Mat &img,Mat &dst)
{
       vector<Mat> channels;
       split(img,channels);//分离通道
       threshold(channels[0],dst, 80, 255, THRESH_BINARY);      
       //threshold(channels[2],dst,0,255,THRESH_OTSU);
       return 0;
}

int ImgPreProcess_ARMER(Mat &img,Mat &dst)
{
       vector<Mat> channels;
       Mat threash;
       split(img,channels);//分离通道
       Mat blueImg=channels.at(0)-channels.at(2);
       threshold(blueImg,dst, 80, 255, THRESH_BINARY);
       //Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
       //dilate(threash, dst, element);
       //threshold(blueImg,dst,0,255,THRESH_OTSU);
       return 0;
}

int ImgPreProcess_ENERGY(Mat &img,Mat &dst)
{
       vector<Mat> channels;
       split(img,channels);//分离通道
       Mat greenImg=channels.at(1)-channels.at(2);
       threshold(greenImg,dst,0,255,THRESH_OTSU);
       return 0;
}
#if 0
#include <ctime>
int gama = 50;
// static void on_ContrastandBright(int,void *);
// int contrastvalue=80;//对比度和亮度都初始化为80
// int brightness=80;

int ImgPreProcess(Mat &img,Mat &dst,Size &imageSize)
{
       //clock_t startTime1 = clock();
       //Mat calibratedImg;
       //remap(img, calibratedImg, map1, map2, INTER_NEAREST);//对每一帧图像进行消歧
       //clock_t endTime1 = clock();
       //cout << "消歧用时："  << double(endTime1 - startTime1) / CLOCKS_PER_SEC << "s" << endl;
       vector<Mat> channels;
       split(img,channels);//分离通道
       //Mat Blur,threashould;
       //GaussianBlur(channels[2],Blur,Size(7,7),0,0);//高斯滤波降噪
       clock_t startTime2 = clock();
       threshold(channels[2],dst,0,255,THRESH_OTSU);
       clock_t endTime2 = clock();
       cout << "OTSU用时："  << double(endTime2 - startTime2) / CLOCKS_PER_SEC << "s" << endl;
       imshow("preproccesed",dst);
       return 0;
}
#endif


