////////////////Object detect/////////////////////////
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include"opencv2/calib3d/calib3d.hpp"
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/src/Core/DiagonalMatrix.h>
#include <eigen3/Eigen/Dense>  //稠密矩阵的代数运算
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include "/home/lijialiang/xbotcon/include/distancedetection.h"
#include "/home/lijialiang/xbotcon/include/kalman_filter.h"
#include "/home/lijialiang/xbotcon/include/getTargtet2dPosition.h"
#include "/home/lijialiang/xbotcon/include/objClassifier.h"
#include "/home/lijialiang/xbotcon/include/ImgPreprocess.h"
#include "/home/lijialiang/xbotcon/include/color_picker.h"

using namespace cv;
using namespace std;

VideoCapture cap("/home/lijialiang/视频/装甲板测试视频.avi");

vector<RotatedRect> rect;//
const int imageWidth = 640; //定义图片大小，即摄像头的分辨率
const int imageHeight = 480;
const int ARMERMODE=0;
const int ENERGYMODE=1;
vector<RotatedRect> armer_rect;
vector<RotatedRect> energy_rect;
vector<Point2f> armer_center;

//git
///////////////////////////////////////////////////////////////////
int main() 
{
	Mat img;
	Size imageSize = Size(imageWidth, imageHeight);
	vector< vector<Point2f>> armer_refer_imgPoint;//对应的装甲板2D图像点
	vector< vector<Point2f> > energy_refer_imgPoint;//对应的能量条2D图像点
	long int frame=0;
	while (true)
	{
		cap >> img;
		if(img.empty())
			break;
		++frame;
		cout<<"\n这是第"<<frame<<"帧"<<endl;
		//color_picker(img);	
		imshow("origin",img);
		Mat dst;
		ImgPreProcess(img,dst,imageSize,ARMERMODE);
		bool success=armerClassifier(dst,img,armer_rect,armer_center);
		if(success)
		{
			getTarget2dPosition(armer_rect[armer_rect.size()-1],armer_refer_imgPoint,Point2f(0,0));
			distancedetection(armer_refer_imgPoint,ARMERMODE);
			kalman_filter(armer_center);
		}
		else
		{
			success=energyClassifier(dst,img,energy_rect);
			if(success)
			{
				getTarget2dPosition(energy_rect[energy_rect.size()-1],energy_refer_imgPoint,Point2f(0,0));
				distancedetection(energy_refer_imgPoint,ENERGYMODE);
			}
		}
		waitKey(100);
	}
	return 0;
}





