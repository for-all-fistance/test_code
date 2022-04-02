#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include"opencv2/calib3d/calib3d.hpp"
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/src/Core/DiagonalMatrix.h>
#include <eigen3/Eigen/Dense>  //稠密矩阵的代数运算
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iostream>
using namespace cv;
using namespace std;
const Mat cameraMatrix = (Mat_<double>(3, 3) << 1264.959862205175, 0, 476.6700262090827,
	0, 1274.866344014577, 304.2392983377306,
	0, 0, 1);//外参
const Mat distCoeff = (Mat_<double>(1, 5) << -0.6189680928486849, 7.658300007772248, 0.0121491961772888, 0.0166097829564812, -31.52891832335384);//内参：畸变矩阵


float distancedetection(vector<vector<Point2f>> refer_imgPoint,int mode)
{
	Mat rvecs;//旋转向量
	Mat tvecs;//平移向量
	vector<Point3f> refer_objectPoints;//自定义的3D坐标点
	Mat rmats;//旋转矩阵
	//Mat tmats;//平移矩阵
	if(mode==0)
	{
		refer_objectPoints.push_back(Point3f(0, 0, 0));
		refer_objectPoints.push_back(Point3f(0, 7.4, 0));
		refer_objectPoints.push_back(Point3f(7.4, 7.4, 0));
		refer_objectPoints.push_back(Point3f(7.4, 0, 0));
	}
	else
	{
		refer_objectPoints.push_back(Point3f(0, 0, 0));
		refer_objectPoints.push_back(Point3f(0, 7.4, 0));
		refer_objectPoints.push_back(Point3f(7.4, 7.4, 0));
		refer_objectPoints.push_back(Point3f(7.4, 0, 0));
	}
	solvePnP(refer_objectPoints, refer_imgPoint[refer_imgPoint.size()-1], cameraMatrix, distCoeff, rvecs, tvecs);//solvepnp解算旋转向量和平移向量
	Rodrigues(rvecs, rmats);
	Eigen::Matrix3f R_n;
	Eigen::Vector3f T_n;
	cv2eigen(rmats, R_n);
	cv2eigen(tvecs, T_n);
	Eigen::Vector3f P_oc;
	P_oc = -R_n.inverse()*T_n;
	cout<<"当前距离为："<<"["<<P_oc<<"]"<<endl;
	float distance=sqrt(pow(P_oc.x(),2)+pow(P_oc.y(),2)+pow(P_oc.z(),2));
	rmats.release();//旋转矩阵
	return(distance);
}