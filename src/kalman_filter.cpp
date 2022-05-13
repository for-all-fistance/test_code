//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "/home/pi/Desktop/xbotcon/include/head.h"
#include <iostream>
using namespace std;
using namespace cv;
const int stateNum=4;                                      //状态值10×1向量(x,y,weight,height,angle,△x,△y......)
const int measureNum=4;                                    //测量值10×1向量(x,y,weight,height,angle,△x,△y......)	                               
KalmanFilter KF(stateNum, measureNum, 0);					//创建一个卡尔曼滤波器，****无需重复定义****
/////////////////////kalman filter params////////////////////
const Mat cameraMatrix = (Mat_<double>(3, 3) <<  987.8162802728854, 0, 515.7196277867043,
 0, 999.9341934371877, 383.1530462255868,
 0, 0, 1);//外参
const Mat distCoeff = (Mat_<double>(1, 5) <<  0.2181693038172464, -0.5750471376539981, -0.00785331353651738, 0.0046813462975, 0.3820630120717159 );//内参：畸变矩阵


void kalman_filter(vector<armer> &armers)
{
	KF.transitionMatrix = (Mat_<float>(4,4) <<1,0,1,0,
										 0,1,0,1,
										 0,0,1,0,
										 0,0,0,1);                              //转移矩阵A
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F); //初始测量值x'(0)为0矩阵，因为后面要更新这个值，所以必须先定义
	setIdentity(KF.measurementMatrix);       //测量矩阵H    setIdentity()创建初始值为单位阵
	setIdentity(KF.processNoiseCov, Scalar::all(1e-3));     //系统噪声方差矩阵Q  Q越大越相信观测值
	setIdentity(KF.measurementNoiseCov, Scalar::all(5e-5)); //测量噪声方差矩阵R  R越小越信任观测值
	setIdentity(KF.errorCovPost, Scalar::all(1));   //后验错误估计协方差矩阵P  
	///////////////////KF预测矩形中心点位置/////////////////
		//vector<Point2f>::iterator ite=rect.end();
		//measurement.at<float>(0) = ite->x;
		//measurement.at<float>(1) = ite->y;
		//measurement.at<float>(2)=ite->x-(ite-1)->x;
		//measurement.at<float>(3)=ite->y-(ite-1)->y;
		//undistortPoints(rect,rect,cameraMatrix,distCoeff);
		int size=armers.size()-1;
		measurement.at<float>(0) = armers[size].armer_center.x;
		measurement.at<float>(1) = armers[size].armer_center.y;
		measurement.at<float>(2)=armers[size].armer_center.x-armers[size-1].armer_center.x;
		measurement.at<float>(3)=armers[size].armer_center.y-armers[size-1].armer_center.y;
		//update
		KF.correct(measurement);
		//kalman prediction
		KF.predict();
		armers.back().point_pre=Point2f(KF.statePre.at<float>(0),KF.statePre.at<float>(1));
		cout<<"预测下一帧装甲板位于：["<<KF.statePre.at<float>(0)<<","<<KF.statePre.at<float>(1)<<"]"<<endl;

}
