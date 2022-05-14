#ifndef _PARAMS_H
#define _PARAMS_H
using namespace cv;
using namespace std;

const int imageWidth = 640; //定义图片大小，即摄像头的分辨率
const int imageHeight = 480;
const float SPEED_BULLET =14;//m/s
const float REAL_HEIGHT_ARMER=0.07;//7cm
const Mat cameraMatrix = (Mat_<double>(3, 3) <<  611.6348658352435, 0, 317.0063484473284,
0, 618.2053424684727, 238.8843698515634,
 0, 0, 1);//外参
const Mat distCoeff = (Mat_<double>(1, 5) << 0.1252247200301222, 0.3344879529935187, 0.004913024896752389, -0.004298159448975511, -2.743281072337644 );//内参：畸变矩阵
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
#endif
