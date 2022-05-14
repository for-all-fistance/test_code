#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "include/head.h"
#include "include/params.h"
#include "include/DEBUG.h"
#include "include/threads.h"
#include <vector>
#include <stdio.h>
#include <cmath>
#include <ctime>//测试用
#include <iostream>
using namespace cv;
using namespace std;

mutex mutex_fd;
mutex mutex_send;
mutex mutex_cap;
mutex mutex_img;
mutex mutex_stand_by;
condition_variable cv_send;
condition_variable cv_cap;
bool stop=false;


Point2d angle_solver(Point2f &P_oc)
{
	static double cx=cameraMatrix.at<double>(0,2);
	static double cy=cameraMatrix.at<double>(1,2);
	static double fx=cameraMatrix.at<double>(0,0);
	static double fy=cameraMatrix.at<double>(1,1);
	double rx=(P_oc.x-cx)/fx;
    double ry=(P_oc.y-cy)/fy;
    return(Point2f(atan(rx)/CV_PI*180,atan(ry)/CV_PI*180));
}

///////////////////////////////////////////////////////////////////
int main() 
{
	Mat img;
	Size imageSize = Size(imageWidth, imageHeight);
	vector<RotatedRect> energy_rect;
	vector< vector<Point2f>> energy_refer_imgPoint;//对应的能量条2D图像点
	Point2f point_angle;
	///////////test//////////
	double time_all=0;
	int frame=0;
	//////////test//////////
	char command='~';
	bool success =false;
	vector<armer> armers;
	armer init_armer {Point2f(320,240),{},0,0,{},Rect(),{},{},Point2f(0,0),Point2f(0,0),Point2f(0,0)};
	armers.push_back(init_armer);
	int fd=serial_init();
	if(fd==-1)
		return -1;
	thread get_th=thread(get_thread,std::ref(command),fd,std::ref(stop));
	thread send_th=thread(send_thread,fd,std::ref(point_angle),std::ref(stop));
	thread cap_th=thread(cap_thread,std::ref(img),std::ref(stop));
	waitKey(1000);
	while (frame<100)
	{
		//cap >> img;
		clock_t startTime = clock();
		if(img.empty())
			{
				cout<<"no img ,error"<<endl;
				return -1;
			}
		if(command=='~')
		{
			cout<<"stand by"<<endl;
			waitKey(40);//wait 40ms for the next command
		}
		else if(command=='%')
		{
			
			unique_lock <mutex> lock_cap(mutex_cap);
			cv_cap.wait(lock_cap);
			Mat dst;
			//if(success)
				//{
					//SetROI(img,armer_real_position.back());
					//imshow("ROI",img);
				//}
			unique_lock <mutex> lock_img(mutex_img);
			ImgPreProcess_ARMER(img,dst);
			lock_img.unlock();
			clock_t startTime1 = clock();
			success=armerClassifier(dst,armers);
			clock_t endTime1 = clock();
			cout << "classify用时："  << double(endTime1 - startTime1) / CLOCKS_PER_SEC << "s" << endl;
			if(success)
			{
				getTarget2dPosition(armers,Point2f(0,0));
				undistortPoints(armers.back().armer_refer_imgPoint,armers.back().armer_refer_imgPoint,cameraMatrix,distCoeff,noArray(),cameraMatrix);
				armers.back().armer_center=(armers.back().armer_refer_imgPoint[1]+armers.back().armer_refer_imgPoint[3])/2.0;
				circle(img,armers.back().armer_center,5,Scalar(120,200,0),FILLED);
				gravity_offset_composite(armers);
				kalman_filter(armers);
				circle(img,armers.back().point_pre,5,Scalar(120,200,0),FILLED);
				unique_lock <mutex> lock(mutex_send);
				point_angle=angle_solver(armers.back().armer_center);
				cv_send.notify_one();
			}
		}
		#if 0
		else if(command=='&')
		{
			Mat dst;
			ImgPreProcess_ENERGY(img,dst);
			bool success=energyClassifier(dst,img,energy_rect);
			if(success)
			{
				getTarget2dPosition(energy_rect.back(),energy_refer_imgPoint,Point2f(0,0));
				distancedetection_energy(energy_refer_imgPoint.back());
				vector<Point2f> target_center={energy_rect.back().center};
				undistortPoints(target_center,target_center,cameraMatrix,distCoeff);
				point_angle=angle_solver(target_center.at(0));
				cv_send.notify_one();
			}
		}
		#endif
		clock_t endTime = clock();
		double this_time=double(endTime - startTime) / CLOCKS_PER_SEC;
		time_all+=this_time;
		++frame;
		cout << "该帧用时：" << this_time << "s" << endl;
		//imshow("img",img);
		//waitKey(1);
	}
	stop=true;
	mutex_send.unlock();
	mutex_fd.unlock();
	mutex_img.unlock();
	cv_send.notify_all();
	get_th.join();
	send_th.join();
	cap_th.join();
	serialClose(fd);
	cout<<"该次运行平均时间:"<<time_all/double(frame)<<endl;
	return 0;
}

void SetROI(Mat& img,const Point2f& armer_real_position)
{
		float x=armer_real_position.x,y=armer_real_position.y;
		float collom_begin=(x>200)?200:x,row_begin=(y>150)?150:y,collom_end=(330>y)?150:(480-y),row_end=(440>x)?200:(640-x);
		img=img(Range(y-row_begin,y+row_end),Range(x-collom_begin,x+collom_end));
	//imshow("roi img",img);
}
