#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "head.h"
using namespace std;
using namespace cv;



void getTarget2dPosition(armer& this_armer, const cv::Point2f& offset)
{
	vector<Point2f> temp;
	Rect2f rect=this_armer.armer_rect;
	temp.push_back(rect.tl()+Point2f(0,rect.height)+offset);//bl
	temp.push_back(rect.br()+offset);//br
	temp.push_back(rect.tl()+Point2f(rect.width,0)+offset);//tr
	temp.push_back(rect.tl()+offset);//tl
	this_armer.armer_refer_imgPoint=temp;
}
