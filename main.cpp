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
#include "include/head.h"
using namespace cv;
using namespace std;

// VideoCapture cap("/home/lijialiang/视频/装甲板测试视频.avi");
VideoCapture cap(0);
vector<RotatedRect> rect;//
const int imageWidth = 720; //定义图片大小，即摄像头的分辨率
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
	int frame=0;
	float pre_error_x_total=0;
	float pre_error_y_total=0;
	int armer_count=0;
	vector<Point2f> point_pre;
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
			armer_count++;
			if(armer_count>2)
				{
					pre_error_x_total+=abs(armer_center.back().x-point_pre.back().x);
					cout<<"平均x坐标误差为："<<pre_error_x_total/armer_count<<endl;
					pre_error_y_total+=abs(armer_center.back().y-point_pre.back().y);
					cout<<"平均y坐标误差为："<<pre_error_y_total/armer_count<<endl;
				}
			getTarget2dPosition(armer_rect[armer_rect.size()-1],armer_refer_imgPoint,Point2f(0,0));
			distancedetection(armer_refer_imgPoint,ARMERMODE);
			point_pre.push_back(kalman_filter(armer_center));
			position_adjust(point_pre.back());
		}
		else
		{
			cout<<"没有找到装甲板"<<"\n开始检测能量条"<<endl;
			success=energyClassifier(dst,img,energy_rect);
			if(success)
			{
				getTarget2dPosition(energy_rect[energy_rect.size()-1],energy_refer_imgPoint,Point2f(0,0));
				distancedetection(energy_refer_imgPoint,ENERGYMODE);
			}
		}
		waitKey(100);
		cout<<"当前装甲板识别&匹配成功率为："<<(float(armer_count)/float(frame))*100<<"%"<<endl;
	}
	return 0;
}





