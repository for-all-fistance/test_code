using namespace cv;
using namespace std;
float distancedetection(vector<vector<Point2f>> refer_imgPoint,int mode);
int color_picker(Mat img);
void getTarget2dPosition(const cv::RotatedRect& rect, std::vector<std::vector<Point2f>> &target2d, const cv::Point2f& offset) ;
int ImgPreProcess(Mat &img,Mat &dst,Size &imageSize,int mode);
Point2f kalman_filter(vector<Point2f> rect);
int armerClassifier(Mat &img,Mat origin,vector<RotatedRect> &armer_rect,vector<Point2f> &armer_center);
int energyClassifier(Mat &img,Mat origin,vector<RotatedRect> &energy_rect);
int position_adjust(Point2f &);