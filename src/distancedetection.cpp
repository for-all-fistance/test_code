#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/DiagonalMatrix.h>
#include <eigen3/Eigen/Dense>  //稠密矩阵的代数运算
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iostream>
#include <ctime>//测试用
#include "params.h"
#include "head.h"
using namespace cv;
using namespace std;


float distancedetection_armer(vector<Point2f> &refer_imgPoint)
{
	Mat rvecs;//旋转向量
	Mat tvecs;//平移向量
	vector<Point3f> refer_objectPoints;//自定义的3D坐标点
	Mat rmats;//旋转矩阵
	refer_objectPoints.push_back(Point3f(-40, -22.5, 0));
	refer_objectPoints.push_back(Point3f(40, -22.5, 0));
	refer_objectPoints.push_back(Point3f(40, 22.5, 0));
	refer_objectPoints.push_back(Point3f(-40, 22.5, 0));
	solvePnP(refer_objectPoints, refer_imgPoint, cameraMatrix, distCoeff, rvecs, tvecs,true,SOLVEPNP_EPNP);//solvepnp解算旋转向量和平移向量
	Rodrigues(rvecs, rmats);
	Eigen::Matrix3f R_n;
	Eigen::Vector3f T_n;
	cv2eigen(rmats, R_n);
	cv2eigen(tvecs, T_n);
	Eigen::Vector3f P_oc;
	P_oc = -R_n.inverse()*T_n;
	//cout<<"当前3D位姿坐标为："<<"["<<P_oc<<"]"<<endl;
	return(P_oc.z()*0.0001);//m
}

void gravity_offset_composite(armer &this_armers)
{
	float distance=distancedetection_armer(this_armers.armer_refer_imgPoint);
	float gravity_offset=(9.8*distance*distance*this_armers.height)/(2*SPEED_BULLET*SPEED_BULLET*REAL_HEIGHT_ARMER);
	this_armers.armer_center.y-=gravity_offset;
}


float distancedetection_energy(vector<Point2f> refer_imgPoint)
{
	vector<Point2f> refer_imgPoint_out;
	undistortPoints(refer_imgPoint,refer_imgPoint_out,cameraMatrix,distCoeff);
	Mat rvecs;//旋转向量
	Mat tvecs;//平移向量
	vector<Point3f> refer_objectPoints;//自定义的3D坐标点
	Mat rmats;//旋转矩阵
	refer_objectPoints.push_back(Point3f(0, 0, 0));
	refer_objectPoints.push_back(Point3f(0, 1.6, 0));
	refer_objectPoints.push_back(Point3f(7, 1.6, 0));
	refer_objectPoints.push_back(Point3f(7, 0, 0));
	solvePnP(refer_objectPoints, refer_imgPoint_out, cameraMatrix, distCoeff, rvecs, tvecs,true,SOLVEPNP_EPNP);//solvepnp解算旋转向量和平移向量
	
	Rodrigues(rvecs, rmats);
	Eigen::Matrix3f R_n;
	Eigen::Vector3f T_n;
	cv2eigen(rmats, R_n);
	cv2eigen(tvecs, T_n);
	Eigen::Vector3f P_oc;
	P_oc = -R_n.inverse()*T_n;
	//cout<<"当前3D位姿坐标为："<<"["<<P_oc<<"]"<<endl;
	return(P_oc.z());
}

