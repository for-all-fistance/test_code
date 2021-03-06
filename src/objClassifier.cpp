#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include "head.h"
#include "params.h"
using namespace std;
using namespace cv;



inline float distance_calc(RotatedRect a,RotatedRect b)
{
    float x1=a.center.x;
    float y1=a.center.y;
    float x2=b.center.x;
    float y2=b.center.y;
    return(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}

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
            return 1;
        }
    return 0;
}
int armerClassifier(Mat &img,armer &this_armer,vector<armer>& armers)
{
    vector<Vec4i> hierarchy;//储存边界的拓扑信息，如前一个轮廓，后一个轮廓，父轮廓等
    vector<vector<Point>> contours;//储存边界信息,不能使用point2f类型，contours都是point(int)类型
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//寻找边界
    int count=0;
    //cout<<"size:"<<contours.size()<<endl;
    //armer this_armer;
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
                this_armer.height=minAreaRect_armer.size.height;
                this_armer.width=minAreaRect_armer.size.width;
                if(this_armer.height<=this_armer.width)
                    swap(this_armer.height,this_armer.width);
                float lenthRate=this_armer.height/this_armer.width;//矩形长宽比
                //cout<<"lenthRate:"<<lenthRate<<endl;
                if(lenthRate>MINLENTHRATE_ARMER&&lenthRate<MAXLENTHRATE_ARMER)
                {
                    vector<vector<Point>> HullPoints(contours.size());
                    convexHull(Mat(contours[i]),HullPoints[i],false);//寻找凸包
                    float solidity=contourArea(HullPoints[i])/area;//凸度
                    //cout<<"solidity:"<<solidity<<endl;
                    if(solidity>MINSOLIDITY_ARMER)
                    {
                        this_armer.armer_light.push_back(minAreaRect_armer);
                        ++count;
                    }
                }
            }
        }
    }
    //cout<<"count"<<count<<endl;
    if(count>1)
    {
        return pair_armer(armers,count,this_armer);
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


int armerClassifier(Mat &img,Mat origin,vector<Point2f> &armer_center)
{
    vector<RotatedRect> armer_rect;
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
        cout<<"area:"<<area<<endl;
        cout<<"areaRate:"<<areaRate<<endl;
        cout<<"lenthRate:"<<lenthRate<<endl;
        cout<<"solidity:"<<solidity<<endl;
        if(area>200&&area<250000/*去除整个图片的边框！*/&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
        {
            armer_rect.push_back(minAreaRect(contours[i]));
            imshow("armer img",img);
            count++;
            Point2f this_armer[4];
            armer_rect[armer_rect.size()-1].points(this_armer);
            for(int i=0;i<4;++i)
            {
                line(origin,this_armer[i],this_armer[(i+1)%4],Scalar(120,200,0),5);
            }
            if(count>1)
            {
                cout<<"成功找到两个符合条件的灯条，开始进行灯条匹配"<<endl;
                for(int j=2;j<=3;++j)
                {
                    float error=distance_calc(armer_rect[armer_rect.size()-1],armer_rect[armer_rect.size()-j])/width;
                    cout<<"error:"<<error<<endl;
                    if(error<MAX_DISTANCE_RATE_ERROR&&error>MIN_DISTANCE_RATE_ERROR)
                    {
                        armer_center.push_back((armer_rect[armer_rect.size()-1].center+armer_rect[armer_rect.size()-j].center)/2);//获取图形的中心，近似等于两个灯条中心连线的中点
                        cout<<"匹配成功！\n"<<"装甲板位于"<<armer_center[armer_center.size()-1]<<endl;            
                        circle(origin,armer_center[armer_center.size()-1],5,Scalar(120,200,0),FILLED);
                        return 1;
                    }
                }
            }
            imshow("armer_found",origin);
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
        Point2f this_armer[4];
        minAreaRect(contours[i]).points(this_armer);
        for(int i=0;i<4;++i)
        {
            line(origin,this_armer[i],this_armer[i%4],(149,255,0));
        }
        imshow("energy_found",img);
        if(area>200&&areaRate>MINAREARATE_ARMER&&areaRate<MAXAREARATE_ARMER&&lenthRate>MINLENTHRATE_ARMER&&solidity>MINSOLIDITY_ARMER)
            {
                energy_rect.push_back(minAreaRect(contours[i]));
                count++;
            }
    }
    if(count>1)//能量块的攻击从上到下
        sort_energy(energy_rect,count);
    return 0;
}
#endif

