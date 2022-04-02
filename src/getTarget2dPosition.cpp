#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;



void getTarget2dPosition(const cv::RotatedRect &rect, vector<vector<Point2f>> &target2d, const cv::Point2f& offset) 
{
	Point2f vertices[4];
	rect.points(vertices);//把矩形的四个点复制给四维点向量
	Point2f lu, ld, ru, rd;
	sort(vertices, vertices + 4, [](const Point2f& p1, const Point2f& p2) { return p1.x < p2.x; });//从4个点的第一个到最后一个进行排序
	if (vertices[0].y < vertices[1].y) {
		lu = vertices[0];
		ld = vertices[1];
	}
	else {
		lu = vertices[1];
		ld = vertices[0];
	}
	if (vertices[2].y < vertices[3].y) {
		ru = vertices[2];
		rd = vertices[3];
	}
	else {
		ru = vertices[3];
		rd = vertices[2];
	}
	vector<Point2f> temp;
	temp.push_back(lu+offset);
	temp.push_back(ru+offset);
	temp.push_back(rd+offset);
	temp.push_back(ld+offset);
	target2d.push_back(temp);
}