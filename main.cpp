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
#include "include/DEBUG.h"
#include <vector>
#include <stdio.h>
#include <cmath>
#if 1
#include <ctime>//测试用
#include <iostream>
#endif
using namespace cv;
using namespace std;


VideoCapture cap(0);
const int imageWidth = 640; //定义图片大小，即摄像头的分辨率
const int imageHeight = 480;
vector<RotatedRect> armer_rect;//储存识别到的装甲板灯条位置
vector<RotatedRect> energy_rect;//储存能量块位置
vector<Point2f> armer_center;//匹配后的装甲板中心位置
bool success =false;
const Mat cameraMatrix = (Mat_<double>(3, 3) <<  987.8162802728854, 0, 515.7196277867043,
 0, 999.9341934371877, 383.1530462255868,
 0, 0, 1);//外参
const Mat distCoeff = (Mat_<double>(1, 5) <<  0.2181693038172464, -0.5750471376539981, -0.00785331353651738, 0.0046813462975, 0.3820630120717159 );//内参：畸变矩阵
mutex mutex_fd;
mutex mutex_send;
std::condition_variable cv_send;
void get_thread(char& command,int fd)//获取指令的线程
{
	while(true)
	{
		unique_lock<mutex> lock_fd(mutex_fd);
		if(serialDataAvail(fd) >= 1)    //如果串口缓存中有数据
		{
			command=serialGetchar(fd);
			cout<<"get command"<<command<<endl;
			serialFlush(fd); //avoid  repeat reception
		}
		lock_fd.unlock();
		this_thread::sleep_for(std::chrono::milliseconds(50));//等待五十毫秒
	}
}
void send_thread(int fd,Point2d& point_angle)//向下位机发送数据线程
{
	char *message;
	while(true)
	{
		unique_lock <mutex> lock(mutex_send);
		cv_send.wait(lock);
		message=new char[5];
		snprintf(message,5,"%d",int(point_angle.x*1000));
		communicate("x",fd);
		communicate(message,fd);
		communicate("y",fd);
		snprintf(message,5,"%d",int(point_angle.y*1000));
		communicate(message,fd);
		delete message;
	}
}

Point2d angle_solver(Point2f &P_oc)//解算角度
{
	static double cx=cameraMatrix.at<double>(0,2);
	static double cy=cameraMatrix.at<double>(1,2);
	static double fx=cameraMatrix.at<double>(0,0);
	static double fy=cameraMatrix.at<double>(1,1);
	double rx=(P_oc.x-cx)/fx;
    double ry=(P_oc.y-cy)/fy;
    return(Point2d(atan(rx)/CV_PI*180,atan(ry)/CV_PI*180));
}
///////////////////////////////////////////////////////////////////
int main() 
{
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	//cap.set(CAP_PROP_FRAME_HEIGHT,960);
	//cap.set(CAP_PROP_FRAME_WIDTH,720);
	/////////test//////////////
	Mat img;
	Size imageSize = Size(imageWidth, imageHeight);
	vector< vector<Point2f>> armer_refer_imgPoint;//对应的装甲板2D图像点
	vector< vector<Point2f>> energy_refer_imgPoint;//对应的能量条2D图像点
	vector<Point2d> point_pre;
	Point2d point_angle;
	char command='~';
	char *message;
	//cout<<wiringPiSetup()<<endl;
	if(-1==wiringPiSetup())
		{
			cout<<"serial error"<<endl;
			return -1;
		}
	int fd =serialOpen("/dev/ttyAMA0",115200);   //这里波特率设置为115200
	if(fd==-1)
		{
			cout<<"serial open error"<<endl;
			return -1;
		}
	thread get_th=thread(get_thread,std::ref(command),fd);
	thread send_th=thread(send_thread,fd,std::ref(point_angle));
	while (true)
	{
		clock_t startTime = clock();//test
		cap >> img;
		if(img.empty())
			{
				cout<<"no img ,error"<<endl;
				break;
			}
		if(command=='~')//等待开始射击
		{
			cout<<"stand by"<<endl;
			waitKey(40);//wait 40ms for the next command
		}
		else if(command=='%')//开始射击装甲板
		{
			Mat dst;
			clock_t startTime1 = clock();
			if(success)
				{
					//SetROI(img,armer_center.back());
					//imshow("ROI",img);
				}
			ImgPreProcess_ARMER(img,dst);
			clock_t endTime1 = clock();
			cout << "preprocess用时："  << double(endTime1 - startTime1) / CLOCKS_PER_SEC << "s" << endl;
			success=armerClassifier(dst,img,armer_rect,armer_center);
			if(success)
			{
				getTarget2dPosition(armer_rect.back(),armer_refer_imgPoint,Point2f(0,0));//将armer_rect 中的点按顺序排列
				//Point2f temp=armer_center.back();
				//temp.y+=distancedetection_armer(armer_refer_imgPoint.back());//线性修正
				//point_pre.push_back(angle_solver(kalman_filter(armer_center)));
				distancedetection_armer(armer_refer_imgPoint.back());//测距，暂时无用
				vector<Point2f> target_center={Point2f(armer_center.back().x+320,armer_center.back().y+240)};
				undistortPoints(target_center,target_center,cameraMatrix,distCoeff,noArray(),cameraMatrix);//消畸
				cout<<armer_center.back()<<endl;
				unique_lock <mutex> lock(mutex_send);
				point_angle=angle_solver(target_center.at(0));
				cout<<point_angle<<endl;
				cv_send.notify_one();//唤醒被阻塞的send_th线程
				
			}
		}
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
		
		waitKey(100);
		clock_t endTime = clock();
		cout << "该帧用时：" << double(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}
	get_th.join();
	send_th.join();
	serialClose(fd);
	return 0;
}

void SetROI(Mat& img,const Point2f& armer_center)
{
		float x=armer_center.x,y=armer_center.y;
		float collom_begin=(x>200)?200:x,row_begin=(y>150)?150:y,collom_end=(330>y)?150:(480-y),row_end=(440>x)?200:(640-x);
		img=img(Range(y-row_begin,y+row_end),Range(x-collom_begin,x+collom_end));
	//imshow("roi img",img);
}
