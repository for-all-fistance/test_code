//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "params.h"
#include "head.h"
#include <iostream>
using namespace std;
using namespace cv;
const int stateNum=4;                                      //状态值10×1向量(x,y,weight,height,angle,△x,△y......)
const int measureNum=4;                                    //测量值10×1向量(x,y,weight,height,angle,△x,△y......)	                               
KalmanFilter KF(stateNum, measureNum, 0);					//创建一个卡尔曼滤波器，****无需重复定义****


void kalman_filter(vector<armer> &armers,armer this_armer)
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
		measurement.at<float>(0) = this_armer.armer_center.x;
		measurement.at<float>(1) = this_armer.armer_center.y;
		measurement.at<float>(2)=this_armer.armer_center.x-armers.back().armer_center.x;
		measurement.at<float>(3)=this_armer.armer_center.y-armers.back().armer_center.y;
		//update
		KF.correct(measurement);
		//kalman prediction
		KF.predict();
		armers.back().point_pre=Point2f(KF.statePre.at<float>(0),KF.statePre.at<float>(1));

}

void kalman_filter_static(vector<armer> &armers)//different params
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
		//cout<<"预测下一帧装甲板位于：["<<KF.statePre.at<float>(0)<<","<<KF.statePre.at<float>(1)<<"]"<<endl;

}
