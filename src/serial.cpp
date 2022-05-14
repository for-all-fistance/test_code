#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
using namespace std;

int serial_init()
{
	if(-1==wiringPiSetup())
		{
			cout<<"serial error"<<endl;
			return -1;
		}
	int fd =serialOpen("/dev/ttyAMA0",115200);   //这里波特率设置为115200
	if(-1==fd)
		{
			cout<<"serial open error"<<endl;
			return -1;
		}
	return fd;
}
