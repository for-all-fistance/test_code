using namespace cv;
using namespace std;
struct armer//装甲板结构体
{
	Point2f armer_center;//装甲板中心点
	vector<Point2f> armer_coners;//装甲板角点
	float height;//装甲板在图像中的高度
	vector<RotatedRect> armer_light;//装甲板灯条的外接矩形
	Rect2f armer_rect;//装甲板图像
	vector<Point2f> armer_refer_imgPoint;//对应的装甲板2D图像点
	Point2f point_pre;//预测的装甲板中心点
	
};
//Point2f distancedetection_armer(vector<Point2f> refer_imgPoint);
float distancedetection_energy(vector<armer> armers);//计算能量条到相机的距离
void getTarget2dPosition(vector<armer>& armers, const cv::Point2f& offset);//将装甲板的角点按顺序排列
int ImgPreProcess_DEBUG(Mat &img,Mat &dst);
int ImgPreProcess_ARMER(Mat &img,Mat &dst);
int ImgPreProcess_ENERGY(Mat &img,Mat &dst);
void SetROI(Mat& img,const vector<armer> armers);//设置ROI，未使用
void kalman_filter(vector<armer> &armers);//卡尔曼滤波预测下一帧的装甲板位置
int armerClassifier(Mat &img,vector<armer>&);//筛选和配对灯条，得出装甲板在图像中的位置
int energyClassifier(Mat &img,Mat origin,vector<armer> armers);//筛选能量块，未使用
int focused(Point2f target);//判断是否足够对齐，可以击打目标，未使用
void gravity_offset_composite(vector<armer> &armers);//通过pnp解算出得距离，计算子弹下坠距离，并消除误差


