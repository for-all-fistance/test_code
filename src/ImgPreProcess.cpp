#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "DEBUG.h"
#include "params.h"
using namespace cv;
using namespace std;

int ImgPreProcess_DEBUG(Mat &img,Mat &dst)
{
       vector<Mat> channels;
       split(img,channels);//分离通道
       threshold(channels[0],dst, 60, 255, THRESH_BINARY);      
       //threshold(channels[2],dst,0,255,THRESH_OTSU);
       return 0;
}

int ImgPreProcess_ARMER(Mat &img,Mat &dst)
{
       vector<Mat> channels;
       Mat threash;
       split(img,channels);//分离通道
       Mat blueImg=channels.at(0)-channels.at(2);
       //threshold(blueImg,threash,0,255,THRESH_OTSU);
       threshold(blueImg,threash, threash_num, 255, THRESH_BINARY);
       //Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
       //dilate(threash, dst, element);
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



