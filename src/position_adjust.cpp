#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <head.h>
using namespace std;
using namespace cv;

const float width_error=20;
const float height_error=20;
int position_adjust(Point2f &target)
{
    char message[3];
    if(target.x>=360)
        if(target.y>=240)
            {
                message[0]='r';
                message[1]='d';//cout<<"装甲板在右下方"<<endl;
                message[2]='\0';
            }
        else
            {
                message[0]='r';
                message[1]='u';//cout<<"装甲板在右上方"<<endl;
                message[2]='\0';
            }
    else
        if(target.y>=240)
            {
                message[0]='l';
                message[1]='d';//cout<<"装甲板在左下方"<<endl;
                message[2]='\0';
            }
        else
            {
                message[0]='l';
                message[1]='u';//cout<<"装甲板在左上方"<<endl;
                message[2]='\0';
            }
            //communicate(message);
    return 0;
}
int focused(Point2f target)
{ 
    Rect ceter_error(360,240,width_error,height_error);
    if(target.inside(ceter_error))
        return 1;
    return 0;
}


