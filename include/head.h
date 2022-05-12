using namespace cv;
using namespace std;
struct armer//装甲板结构体
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
float distancedetection_energy(vector<armer> armers);
int color_picker(Mat img);
void getTarget2dPosition(vector<armer>& armers, const cv::Point2f& offset);
int ImgPreProcess_DEBUG(Mat &img,Mat &dst);
int ImgPreProcess_ARMER(Mat &img,Mat &dst);
int ImgPreProcess_ENERGY(Mat &img,Mat &dst);
void SetROI(Mat& img,const vector<armer> armers);
void kalman_filter(vector<armer> &armers);
int armerClassifier(Mat &img,vector<armer>&);
int energyClassifier(Mat &img,Mat origin,vector<armer> armers);
int position_adjust(Point2f &);
int focused(Point2f target);
int communicate(char const *message,int const fd);
void gravity_offset_composite(vector<armer> &armers);


