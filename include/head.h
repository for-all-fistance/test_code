#ifndef _HEAD_H
#define _HEAD_H
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
struct armer
{
	Point2f armer_center;
	vector<Point2f> armer_coners;
	float height;
	float width;
	vector<RotatedRect> armer_light;
	Rect2f armer_rect;
	vector<Rect2f> armer_real_position;
	vector<Point2f> armer_refer_imgPoint;//对应的装甲板2D图像点
	Point2f point_pre;
	Point2f point_angle;
	Point2f real_point;
	
};
//Point2f distancedetection_armer(vector<Point2f> refer_imgPoint);
float distancedetection_energy(armer this_armer);
int color_picker(Mat img);
void getTarget2dPosition(armer& this_armers, const cv::Point2f& offset);
int ImgPreProcess_DEBUG(Mat &img,Mat &dst);
int ImgPreProcess_ARMER(Mat &img,Mat &dst);
int ImgPreProcess_ENERGY(Mat &img,Mat &dst);
void SetROI(Mat& img,const vector<armer> armers);
void kalman_filter(vector<armer> &armers,armer& this_armer);
void kalman_filter_static(vector<armer> &armers);
int armerClassifier(Mat &img,armer& this_armer,vector<armer>& armers);
int energyClassifier(Mat &img,Mat origin,vector<armer>& armers);
int position_adjust(Point2f &);
int focused(Point2f target);
void gravity_offset_composite(armer &armers);

////////////new////////
int get_thread(char& command,int fd,bool &stop);
int send_thread(int fd,Point2f &point_angle,bool &stop);
int cap_thread(Mat &img,Mat &dst,bool &stop);
int serial_init();
#endif


