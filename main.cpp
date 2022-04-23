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
#include "include/DEBUG.h"
#include <vector>
#include <ctime>//测试用
using namespace cv;
using namespace std;

VideoCapture cap("/home/lijialiang/视频/装甲板测试视频.avi");
// VideoCapture cap(0);
vector<RotatedRect> rect;//
const int imageWidth = 720; //定义图片大小，即摄像头的分辨率
const int imageHeight = 480;
const int ARMERMODE=0;
const int ENERGYMODE=1;
vector<RotatedRect> armer_rect;
vector<RotatedRect> energy_rect;
vector<Point2f> armer_center;

///////////////////////////////////////////////////////////////////
int main() 
{
	//////////test//////////
	double total_time=0;
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
		clock_t startTime = clock();
		cap >> img;
		if(img.empty())
			break;
		#if DEBUG
		++frame;
		cout<<"\n这是第"<<frame<<"帧"<<endl;	
		imshow("origin",img);
		clock_t startTime1 = clock();
		#endif
		Mat dst;
		ImgPreProcess(img,dst,imageSize,ARMERMODE);
		#if DEBUG
		clock_t endTime1 = clock();
		cout << "预处理用时："  << double(endTime1 - startTime1) / CLOCKS_PER_SEC << "s" << endl;
		#endif
		bool success=armerClassifier(dst,img,armer_rect,armer_center);
		if(success)
		{
			// char message[12];
			// string temp="armer found";
			// for(int i=0;i<=11;++i)
			// 	message[i]=temp[i];
			// communicate(message);
			#if DEBUG
			++armer_count;
			if(armer_count>2)
				{
					pre_error_x_total+=abs(armer_center.back().x-point_pre.back().x);
					cout<<"平均x坐标误差为："<<pre_error_x_total/armer_count<<endl;
					pre_error_y_total+=abs(armer_center.back().y-point_pre.back().y);
					cout<<"平均y坐标误差为："<<pre_error_y_total/armer_count<<endl;
				}
			#endif
			getTarget2dPosition(armer_rect[armer_rect.size()-1],armer_refer_imgPoint,Point2f(0,0));
			distancedetection(armer_refer_imgPoint,ARMERMODE);
			point_pre.push_back(kalman_filter(armer_center));
			// if(focused(point_pre.back()))
			// 	{
			// 		char message[8];
			// 		string temp="focused";
			// 		for(int i=0;i<=7;++i)
			// 		message[i]=temp[i];
			// 		communicate(message);
			// 	}
			#if DEBUG
			position_adjust(point_pre.back());
			#endif
		}
		else
		{
			// char message[16];
			// string temp="armer not found";
			// for(int i=0;i<=15;++i)
			// 	message[i]=temp[i];
			// communicate(message);
			//cout<<"没有找到装甲板"<<"\n开始检测能量条"<<endl;
			success=energyClassifier(dst,img,energy_rect);
			if(success)
			{
				// char message[13];
				// string temp="energy found";
				// for(int i=0;i<=12;++i)
				// 	message[i]=temp[i];
				// communicate(message);
				getTarget2dPosition(energy_rect[energy_rect.size()-1],energy_refer_imgPoint,Point2f(0,0));
				distancedetection(energy_refer_imgPoint,ENERGYMODE);
				// if(focused(point_pre.back()))
				// {
				// 	char message[8];
				// 	string temp="focused";
				// 	for(int i=0;i<=7;++i)
				// 		message[i]=temp[i];
				// 	communicate(message);
				// }
			}
		}
		waitKey(1);
		//cout<<"当前装甲板识别&匹配成功率为："<<(float(armer_count)/float(frame))*100<<"%"<<endl;
		clock_t endTime = clock();
		total_time+=double(endTime - startTime) / CLOCKS_PER_SEC;
		cout << "该帧用时："  << double(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}
	cout<<"平均用时："<<total_time/double(frame)<<endl;
	return 0;
}

