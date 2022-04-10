#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

const Mat cameraMatrix = (Mat_<double>(3, 3) <<  1.2853517927598091e+03, 0., 3.1944768628958542e+02, 0.,
       1.2792339468697937e+03, 2.3929354061292258e+02, 0., 0., 1.);//外参
const Mat distCoeff = (Mat_<double>(1, 5) <<  -6.3687295852461456e-01, -1.9748008790347320e+00,
       3.0970703651800782e-02, 2.1944646842516919e-03, 0. );//内参：畸变矩阵
int gama = 10;
// static void on_ContrastandBright(int,void *);
// int contrastvalue=80;//对比度和亮度都初始化为80
// int brightness=80;

int ImgPreProcess(Mat &img,Mat &dst,Size &imageSize,int mode)
{
/////////////////调曝光和对比度(伽马增益)//////////
       Mat gamaimg;
       createTrackbar("gama","gama",&gama,250);//创建滑动条
    // 构建查找表
	Mat lookUpTable(1, 256, CV_8U);
	uchar* p = lookUpTable.ptr();
	for (int i = 0; i < 256; ++i)
		p[i] = saturate_cast<uchar>(pow(i / 255.0, double(gama)/10) * 255.0);

   // 使用查找表进行对比度亮度调整
	double t3 = (double)getTickCount();
	LUT(img, lookUpTable, gamaimg);
	double time3 = ((double)getTickCount() - t3) / getTickFrequency();
       imshow("gama",gamaimg);
/////////////////////////////////////////////////////
       Mat map1, map2,calibratedImg;
       initUndistortRectifyMap(cameraMatrix, distCoeff, noArray(), cameraMatrix, imageSize, CV_32FC1, map1, map2);//对图像进行消歧
       remap(gamaimg, calibratedImg, map1, map2, INTER_NEAREST);//对每一帧图像进行消歧
       vector<Mat> channels;
       split(calibratedImg,channels);
       Mat Blur,threashould;
       GaussianBlur(channels[2],Blur,Size(7,7),0,0);//高斯滤波降噪
       threshold(Blur,dst,0,255,THRESH_OTSU);
       imshow("preproccesed",dst);
       return 0;
}


