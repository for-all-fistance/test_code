using namespace cv;
using namespace std;
float distancedetection_armer(vector<Point2f> refer_imgPoint);
float distancedetection_energy(vector<Point2f> refer_imgPoint);
int color_picker(Mat img);
void getTarget2dPosition(const cv::RotatedRect& rect, std::vector<std::vector<Point2f>> &target2d, const cv::Point2f& offset) ;
int ImgPreProcess_DEBUG(Mat &img,Mat &dst);
int ImgPreProcess_ARMER(Mat &img,Mat &dst);
int ImgPreProcess_ENERGY(Mat &img,Mat &dst);
void SetROI(Mat& img,const Point2f& armer_center);
Point2f kalman_filter(vector<Point2f> rect);
int armerClassifier(Mat &img,Mat origin,vector<RotatedRect> &armer_rect,vector<Point2f> &armer_center);
int energyClassifier(Mat &img,Mat origin,vector<RotatedRect> &energy_rect);
int position_adjust(Point2f &);
int focused(Point2f target);
int communicate(char const *message,int const fd);

