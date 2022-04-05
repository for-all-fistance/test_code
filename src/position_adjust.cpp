#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int position_adjust(Point2f &target)
{
    if(target.x>=360)
        if(target.y>=240)
            cout<<"装甲板在右下方"<<endl;
        else
            cout<<"装甲板在右上方"<<endl;
    else
        if(target.y>=240)
            cout<<"装甲板在左下方"<<endl;
        else
            cout<<"装甲板在左上方"<<endl;
    return 0;
}


