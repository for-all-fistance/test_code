#include <thread>
#include <mutex>
#include <condition_variable>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <ctime>
#include "head.h"
using namespace std;
using namespace cv;

extern mutex mutex_fd;
extern mutex mutex_send;
extern mutex mutex_cap;
extern mutex mutex_img;
extern condition_variable cv_send;
extern condition_variable cv_cap;
extern mutex mutex_stand_by;

int get_thread(char& command,int fd,bool &stop)
{
	while(stop!=true)
	{
		unique_lock<mutex> lock_fd(mutex_fd);
		if(serialDataAvail(fd) >= 1)    //如果串口缓存中有数据
		{
			command=serialGetchar(fd);
			serialFlush(fd); //avoid  repeat reception
		}
		lock_fd.unlock();
		this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	return 0;
}

int send_thread(int fd,Point2f &point_angle,bool &stop)
{
	string pnt_int;
	while(stop!=true)
	{
		string message;
		stringstream ssx;
		stringstream ssy;
		string str_container;
		unique_lock <mutex> lock(mutex_send);
		cv_send.wait(lock);
		unique_lock<mutex> lock_fd(mutex_fd);
		//cout<<"angle:"<<point_angle<<endl;
		message.push_back('x');
		if (point_angle.x < 0)
			message.push_back('-');
		else
			message.push_back('+');
		ssx << setw(2) << setfill('0') << int(abs(point_angle.x));
		ssx >> pnt_int;
		message += pnt_int;
		message.push_back('@');
		message.push_back('y');
		if (point_angle.y < 0)
			message.push_back('-');
		else
			message.push_back('+');
		ssy << setw(2) << setfill('0') << int(abs(point_angle.y));
		ssy >> pnt_int;
		message += pnt_int;
		message.push_back('@');
		serialPuts(fd,message.c_str());
	}
	return 0;
}


//VideoCapture cap("/home/pi/Desktop/xbotcon/armer test.avi");
VideoCapture cap(0);
int cap_thread(Mat &img,Mat &dst,bool &stop)
{
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	Mat temp;
	while(stop!=true)
	{
		cap>>img;
		unique_lock <mutex> lock_img(mutex_img);
		ImgPreProcess_ARMER(img,dst);
		lock_img.unlock();
		cv_cap.notify_all();
	}
	return 0;
}

