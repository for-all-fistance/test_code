#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<iostream>
using namespace std;
using namespace cv;

const float MINAREARATE_ARMER=1.2;//最小外接矩形与轮廓的面积比值在一定范围内
const float MAXAREARATE_ARMER=1.7;
const float MINLENTHRATE_ARMER=1.3;//长宽比大于一定值
const float MINSOLIDITY_ARMER=0.9;//轮廓的凸度(Solidity)大于一定值
const float MAXSOLIDITY_ARMER=1.5;

const float MAX_DISTANCE_RATE_ERROR=1;//灯条中心点连线与灯条长度的比值误差最大值

const float MINAREARATE_ENERGY=2;
const float MAXAREARATE_ENERGY=2;
const float MINLENTHRATE_ENERGY=1.5;
const float MINSOLIDITY_ENERGY=1;
const float MAXSOLIDITY_ENERGY=1;

inline float distance_calc(RotatedRect a,RotatedRect b)
{
    float x1=a.center.x;
    float y1=a.center.y;
    float x2=b.center.x;
    float y2=b.center.y;
    return((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}


int armerClassifier(Mat &img,Mat oringin,vector<RotatedRect> &armer_rect,vector<Point2f> &armer_center)
{
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息,不能使用point2f类型，contours都是point(int)类型
    findContours(img, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//寻找边界
    //drawContours(img,contours,-1,(23,55,55),5);
    imshow("contours",img);
    for(unsigned int i=0;i<contours.size();++i)
    {
        float area=contourArea(contours[i]);
        float areaRate=minAreaRect(contours[i]).size.area()/area;//最小外接矩形与轮廓的面积比值
        float lenthRate=minAreaRect(contours[i]).size.height/minAreaRect(contours[i]).size.width;//矩形长宽比
        vector<vector<Point>> HullPoints(contours.size());
        convexHull(Mat(contours[i]),HullPoints[i],false);//寻找凸包
        float solidity=contourArea(HullPoints[i])/area;//凸度
        if(area>100&&area<250000)
        {
            cout<<"area:"<<area<<endl;
            cout<<"areaRate:"<<areaRate<<endl;
            cout<<"lenthRate:"<<lenthRate<<endl;
            cout<<"solidity:"<<solidity<<endl;
            Point2f temp[4];
            minAreaRect(contours[i]).points(temp);
            for(int i=0;i<4;++i)
            {
                line(oringin,temp[i],temp[i%4],0,5);
            }
            imshow("armer img",img);
            imshow("armer",oringin);
        }
        if(area>100&&area<250000/*去除整个图片的边框！*/&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
        {
            cout<<"area:"<<area<<endl;
            cout<<"areaRate:"<<areaRate<<endl;
            cout<<"lenthRate:"<<lenthRate<<endl;
            cout<<"solidity:"<<solidity<<endl;
            armer_rect.push_back(minAreaRect(contours[i]));
            int count=0;
            count++;
            if(count>=2)
            {
                for(int i=2;i<contours.size();++i)
                {
                    if((distance_calc(armer_rect[armer_rect.size()-1],armer_rect[armer_rect.size()-i])/armer_rect[armer_rect.size()-1].size.height)<MAX_DISTANCE_RATE_ERROR)
                    {
                            armer_center.push_back((armer_rect[armer_rect.size()-1].center+armer_rect[armer_rect.size()-i].center)/2);//获取图形的中心，近似等于两个灯条中心连线的中点
                            return 1;
                    }
                }
            }

        }
    }

    return 0;
}

int energyClassifier(Mat &img,Mat origin,vector<RotatedRect> &energy_rect)
{
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//寻找边界
    for(int i=0;i<contours.size();++i)
    {
        float area=contourArea(contours[i]);
        float areaRate=minAreaRect(contours[i]).size.area()/area;//最小外接矩形与轮廓的面积比值
        float lenthRate=minAreaRect(contours[i]).size.height/minAreaRect(contours[i]).size.width;//矩形长宽比
        vector<vector<Point>> HullPoints(contours.size());
        convexHull(Mat(contours[i]),HullPoints[i],false);
        float solidity=area/contourArea(HullPoints[i]);
        Point2f temp[4];
        minAreaRect(contours[i]).points(temp);
        for(int i=0;i<4;++i)
        {
            line(origin,temp[i],temp[i%4],(149,255,0));
        }
        //imshow("energy img",img);
        imshow("energy",origin);
        if(area>100&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
            {
                energy_rect.push_back(minAreaRect(contours[i]));
                return 1;
            }
    }
    return 0;
}

