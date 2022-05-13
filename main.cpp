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
#include "platform/inc/vl53l0x_types.h"
#if 1
#include <ctime>//测试用
#include <iostream>
#endif
using namespace cv;
using namespace std;


//VideoCapture cap("/home/pi/Desktop/xbotcon/armer test.avi");
VideoCapture cap(0);
const int imageWidth = 640; //定义图片大小，即摄像头的分辨率
const int imageHeight = 480;
bool success =false;
const Mat cameraMatrix = (Mat_<double>(3, 3) <<  611.6348658352435, 0, 317.0063484473284,
0, 618.2053424684727, 238.8843698515634,
 0, 0, 1);//外参
const Mat distCoeff = (Mat_<double>(1, 5) << 0.1252247200301222, 0.3344879529935187, 0.004913024896752389, -0.004298159448975511, -2.743281072337644 );//内参：畸变矩阵
mutex mutex_fd;
mutex mutex_send;
condition_variable cv_send;
void get_thread(char& command,int fd)//接受指令的线程
{
	while(true)
	{
		unique_lock<mutex> lock_fd(mutex_fd);
		if(serialDataAvail(fd) >= 1)    //如果串口缓存中有数据
		{
			command=serialGetchar(fd);
			cout<<"get command"<<command<<endl;
			serialFlush(fd); //清空缓冲区，防止重复接收
		}
		lock_fd.unlock();
		this_thread::sleep_for(std::chrono::milliseconds(50));//休眠50ms
	}
}
void send_thread(int fd,Point2f &point_angle)//发送数据给下位机的线程
{
	char *message;
	while(true)
	{
		unique_lock <mutex> lock(mutex_send);
		cv_send.wait(lock);
		unique_lock<mutex> lock_fd(mutex_fd);
		message=new char[5];
		snprintf(message,5,"%d",int(point_angle.x*10000));
		serialPuts(fd,"x");  
		serialPuts(fd,message);
		serialPuts(fd,"y");  
		snprintf(message,5,"%d",int(point_angle.y*10000));
		serialPuts(fd,message);;
		delete message;
	}
}

Point2d angle_solver(Point2f &P_oc)//计算角度
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
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	Mat img;
	Size imageSize = Size(imageWidth, imageHeight);
	vector<RotatedRect> energy_rect;
	vector< vector<Point2f>> energy_refer_imgPoint;//对应的能量条2D图像点
	Point2f point_angle;
	vector<armer> armers;
	armer init_armer {Point2f(320,240),{},0,0,{},Rect(),{},{},Point2f(0,0),Point2f(0,0),Point2f(0,0)};
	armers.push_back(init_armer);
	char command='%';
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
		
		cap >> img;
		clock_t startTime = clock();//test
		if(img.empty())
			{
				cout<<"no img ,error"<<endl;
				break;
			}
		if(command=='~')
		{
			cout<<"stand by"<<endl;
			waitKey(40);//wait 40ms for the next command
		}
		else if(command=='%')
		{
			Mat dst;
			clock_t startTime1 = clock();
			ImgPreProcess_ARMER(img,dst);
			clock_t endTime1 = clock();
			cout << "preprocess用时："  << double(endTime1 - startTime1) / CLOCKS_PER_SEC << "s" << endl;
			success=armerClassifier(dst,armers);
			clock_t endTime2 = clock();
			cout << "classifier用时："  << double(endTime2 - endTime1) / CLOCKS_PER_SEC << "s" << endl;
			if(success)
			{
				getTarget2dPosition(armers,Point2f(0,0));
				// for(int i=0;i<4;++i)
				// 	circle(img,armers.back().armer_refer_imgPoint.at(i),5,Scalar(120,200,0),FILLED);
				undistortPoints(armers.back().armer_refer_imgPoint,armers.back().armer_refer_imgPoint,cameraMatrix,distCoeff,noArray(),cameraMatrix);
				armers.back().armer_center=(armers.back().armer_refer_imgPoint[1]+armers.back().armer_refer_imgPoint[3])/2.0;
				gravity_offset_composite(armers);
				kalman_filter(armers);
				unique_lock <mutex> lock(mutex_send);
				point_angle=angle_solver(armers.back().armer_center);
				cout<<"point_angle:"<<point_angle<<endl;
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
		//imshow("img",img);
		//waitKey(1);
		clock_t endTime = clock();
		cout << "该帧用时：" << double(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	}
	get_th.join();
	send_th.join();
	serialClose(fd);
	return 0;
}

void SetROI(Mat& img,const Point2f& armer_real_position)
{
		float x=armer_real_position.x,y=armer_real_position.y;
		float collom_begin=(x>200)?200:x,row_begin=(y>150)?150:y,collom_end=(330>y)?150:(480-y),row_end=(440>x)?200:(640-x);
		img=img(Range(y-row_begin,y+row_end),Range(x-collom_begin,x+collom_end));
	//imshow("roi img",img);
}
