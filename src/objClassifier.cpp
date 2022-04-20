#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include<cmath>
#include<iostream>
using namespace std;
using namespace cv;

const float MINAREARATE_ARMER=1;//最小外接矩形与轮廓的面积比值在一定范围内
const float MAXAREARATE_ARMER=1.5;
const float MINLENTHRATE_ARMER=3.5;//长宽比大于一定值
const float MINSOLIDITY_ARMER=0.9;//轮廓的凸度(Solidity)大于一定值
const float MAXSOLIDITY_ARMER=1.1;

const float MAX_DISTANCE_RATE_ERROR=15;//灯条中心点连线与灯条长度的比值误差最大值
const float MIN_DISTANCE_RATE_ERROR=8;//灯条中心点连线与灯条长度的比值误差最小值

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
    return(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}

int sort_energy(vector<RotatedRect> &energy_rect,int count);
int armerClassifier(Mat &img,Mat origin,vector<RotatedRect> &armer_rect,vector<Point2f> &armer_center)
{
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息,不能使用point2f类型，contours都是point(int)类型
    findContours(img, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//寻找边界
    int count=0;
    cout<<"共找到"<<contours.size()<<"个轮廓"<<endl;
    for(int i=0;i<contours.size();++i)
    {
        float area=contourArea(contours[i]);
        RotatedRect minAreaRect_armer=minAreaRect(contours[i]);
        float areaRate=minAreaRect_armer.size.area()/area;//最小外接矩形与轮廓的面积比值
        float height=minAreaRect_armer.size.height;
        float width=minAreaRect_armer.size.width;
        if(height<=width)
            swap(height,width);
        float lenthRate=height/width;//矩形长宽比
        vector<vector<Point>> HullPoints(contours.size());
        convexHull(Mat(contours[i]),HullPoints[i],false);//寻找凸包
        float solidity=contourArea(HullPoints[i])/area;//凸度
        // cout<<"area:"<<area<<endl;
        // cout<<"areaRate:"<<areaRate<<endl;
        // cout<<"lenthRate:"<<lenthRate<<endl;
        // cout<<"solidity:"<<solidity<<endl;
        if(area>50&&area<250000/*去除整个图片的边框！*/&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
        {
            armer_rect.push_back(minAreaRect(contours[i]));
            imshow("armer img",img);
            count++;
            Point2f temp[4];
            armer_rect[armer_rect.size()-1].points(temp);
            for(int i=0;i<4;++i)
            {
                line(origin,temp[i],temp[(i+1)%4],Scalar(120,200,0),5);
            }
            if(count>1)
            {
                cout<<"成功找到两个符合条件的灯条，开始进行灯条匹配"<<endl;
                for(int j=2;j<=3;++j)
                {
                    float error=distance_calc(armer_rect[armer_rect.size()-1],armer_rect[armer_rect.size()-j])/width;
                    //cout<<"error:"<<error<<endl;
                    if(error<MAX_DISTANCE_RATE_ERROR&&error>MIN_DISTANCE_RATE_ERROR)
                    {
                        armer_center.push_back((armer_rect[armer_rect.size()-1].center+armer_rect[armer_rect.size()-j].center)/2);//获取图形的中心，近似等于两个灯条中心连线的中点
                        cout<<"匹配成功！\n"<<"装甲板位于"<<armer_center[armer_center.size()-1]<<endl;            
                        circle(origin,armer_center[armer_center.size()-1],5,Scalar(120,200,0),FILLED);
                        return 1;
                    }
                }
            }
            imshow("armer",origin);
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
        float areaRate=minAreaRect(contours[i]).size.area()/area;//最小外接矩形与轮廓的面积比值
        float lenthRate=minAreaRect(contours[i]).size.height/minAreaRect(contours[i]).size.width;//矩形长宽比
        vector<vector<Point>> HullPoints(contours.size());
        convexHull(Mat(contours[i]),HullPoints[i],false);
        float solidity=area/contourArea(HullPoints[i]);
        // Point2f temp[4];
        // minAreaRect(contours[i]).points(temp);
        // for(int i=0;i<4;++i)
        // {
        //     line(origin,temp[i],temp[i%4],(149,255,0));
        // }
        //imshow("energy img",img);
        if(area>100&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
            {
                energy_rect.push_back(minAreaRect(contours[i]));
                count++;
            }
    }
    if(count>1)//能量块的攻击从上到下
        sort_energy(energy_rect,count);
    return 0;
}

int sort_energy(vector<RotatedRect> &energy_rect,int count)
{

    for(int i=1;i<=count;++i)
    {
        for(int j=1;j<=count;)
            {
                
                if(energy_rect[energy_rect.size()-i].center.y<=energy_rect[energy_rect.size()-j].center.y)
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
    //     vector<RotatedRect>::iterator target;
    //     for(int i=1;i<count;++i)
    //     {
    //         target=energy_rect.end();
    //         if((*target).center.y>(*(--target)).center.y)
    //             energy_rect.erase(target);
    //         else if(*target).center.y>(*(--target)).center.y)
            
    //     }
}