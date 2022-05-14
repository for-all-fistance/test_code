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
	char *message;
	while(stop!=true)
	{
		unique_lock <mutex> lock(mutex_send);
		cv_send.wait(lock);
		unique_lock<mutex> lock_fd(mutex_fd);
		//cout<<"angle:"<<point_angle<<endl;
		message=new char[5];
		snprintf(message,5,"%d",int(point_angle.x*10000));
		serialPuts(fd,"x");  
		serialPuts(fd,message);  
		serialPuts(fd,"@"); 
		snprintf(message,5,"%d",int(point_angle.y*10000));
		serialPuts(fd,"y");  
		serialPuts(fd,message); 
		serialPuts(fd,"@"); 
		delete message;
	}
	return 0;
}


//VideoCapture cap("/home/pi/Desktop/xbotcon/armer test.avi");
VideoCapture cap(0);
int cap_thread(Mat &img,bool &stop)
{
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	Mat temp;
	while(stop!=true)
	{
		cap>>temp;
		unique_lock <mutex> lock_img(mutex_img);
		img=temp;
		lock_img.unlock();
		cv_cap.notify_all();
	}
	return 0;
}

