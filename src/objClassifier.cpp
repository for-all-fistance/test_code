#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include "/home/pi/Desktop/xbotcon/include/head.h"
using namespace std;
using namespace cv;

const float MINAREARATE_ARMER=0.95;//最小外接矩形与轮廓的面积比值在一定范围内
const float MAXAREARATE_ARMER=1.5;
const float MINLENTHRATE_ARMER=4;//长宽比大于一定值
const float MAXLENTHRATE_ARMER=7;//长宽比小于一定值
const float MINSOLIDITY_ARMER=0.9;//轮廓的凸度(Solidity)大于一定值
const float MAXSOLIDITY_ARMER=1.1;

const float MAX_ANGLE_ERROR=15;//灯条angle误差最大值

const float MINAREARATE_ENERGY=2;
const float MAXAREARATE_ENERGY=2;
const float MINLENTHRATE_ENERGY=1.5;
const float MINSOLIDITY_ENERGY=1;
const float MAXSOLIDITY_ENERGY=1;

int pair_armer(vector<armer>& armers,int count,armer &this_armer)
{
    vector<RotatedRect>::iterator target=this_armer.armer_light.end();
    vector<vector<int>> all_pos;
    vector<float> all_error;
    bool  paired=false;
    int i_small=0;
    int j_small=0;
    int error_small=888;
    for(int i=1;i<count;++i)
    {
        for(int j=i+1;j<count+1;++j)
            {
                float error=abs((target-i)->angle-(target-j)->angle);
                //cout<<"error:"<<error<<endl;
                if(error<MAX_ANGLE_ERROR)
                {
                    if(error_small>error)
                    {
                        error_small=error;
                        i_small=i;
                        j_small=j;
                    }
                    paired=true;
                }
            }
    }
    if(paired)
        {
            Rect2f r1=(*(target-i_small)).boundingRect();
            Rect2f r2=(*(target-j_small)).boundingRect();
            Rect2f all_rect=r1|r2;
            this_armer.armer_rect=all_rect;
            armers.push_back(this_armer);
            return 1;
        }
    return 0;
}
int armerClassifier(Mat &img,vector<armer> &armers)
{
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息,不能使用point2f类型，contours都是point(int)类型
    findContours(img, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//寻找边界
    int count=0;
    //cout<<"size:"<<contours.size()<<endl;
    armer temp;
    for(int i=0;i<contours.size();++i)
    {
        float area=contourArea(contours[i]);
        //cout<<"area"<<area<<endl;
        if(area>20&&area<250000/*去除整个图片的边框！*/)
        {
            RotatedRect minAreaRect_armer=minAreaRect(contours[i]);
            //cout<<"angle:"<<minAreaRect_armer.angle<<endl;
            float areaRate=minAreaRect_armer.size.area()/area;//最小外接矩形与轮廓的面积比值
            //cout<<"arearate:"<<areaRate<<endl;
            if(areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER)
            {
                temp.height=minAreaRect_armer.size.height;
                temp.width=minAreaRect_armer.size.width;
                if(temp.height<=temp.width)
                    swap(temp.height,temp.width);
                float lenthRate=temp.height/temp.width;//矩形长宽比
                //cout<<"lenthRate:"<<lenthRate<<endl;
                if(lenthRate>MINLENTHRATE_ARMER&&lenthRate<MAXLENTHRATE_ARMER)
                {
                    vector<vector<Point>> HullPoints(contours.size());
                    convexHull(Mat(contours[i]),HullPoints[i],false);//寻找凸包
                    float solidity=contourArea(HullPoints[i])/area;//凸度
                    //cout<<"solidity:"<<solidity<<endl;
                    if(solidity>MINSOLIDITY_ARMER)
                    {
                        temp.armer_light.push_back(minAreaRect_armer);
                        ++count;
                    }
                }
            }
        }
    }
    //cout<<"count"<<count<<endl;
    if(count>1)
    {
        return pair_armer(armers,count,temp);
    }
    return 0;
}

#if DEBUG
#include <cmath>
#include <iostream>


int sort_energy(vector<RotatedRect> &energy_rect,int count)
{
    vector<RotatedRect>::iterator target;
    for(int i=1;i<=count;++i)
    {
        for(int j=1;j<=count;)
            {
                target=energy_rect.end();
                if((target-i)->center.y<=(target-j)->center.y)
                {
                    if(j==count)
                        return i;
                    ++j;
                }
                else
                        break;
            }
    }
    return 0;
}

int energyClassifier(Mat &img,Mat origin,vector<RotatedRect> &energy_rect)
{
    int count=0;
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//寻找边界
    for(int i=0;i<contours.size();++i)
    {
        float area=contourArea(contours[i]);
        
        if(area>100&&area<250000)
        {
            RotatedRect minAreaRect_energy=minAreaRect(contours[i]);
            float areaRate=minAreaRect(contours[i]).size.area()/area;//最小外接矩形与轮廓的面积比值
            if(areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER)
            {
                float height=minAreaRect_energy.size.height;
                float width=minAreaRect_energy.size.width;
                if(height<=width)
                swap(height,width);
                float lenthRate=height/width;//矩形长宽比
                if(lenthRate>MINLENTHRATE_ARMER)
                {
                    vector<vector<Point>> HullPoints(contours.size());
                    convexHull(Mat(contours[i]),HullPoints[i],false);
                    float solidity=area/contourArea(HullPoints[i]);
                    if(solidity>MINSOLIDITY_ARMER)
                    {
                        energy_rect.push_back(minAreaRect(contours[i]));
                        count++;
                    }
                }
            }
        }
    }
    
    if(count>1)//能量块的攻击从上到下
        sort_energy(energy_rect,count);
    return 0;
}
#endif