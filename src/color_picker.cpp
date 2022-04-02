#if 1
//////////color picker/////////////////
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int color_picker(Mat img)
{

	Mat imgHSV, mask, imgColor;
	int hmin = 0, smin = 0, vmin = 0;
	int hmax = 179, smax = 255, vmax = 255;
	namedWindow("Trackbars", (640, 200)); // Create Window
	while(1)
	{	

		createTrackbar("Hue Min", "Trackbars", &hmin, 179);
		createTrackbar("Hue Max", "Trackbars", &hmax, 179);
		createTrackbar("Sat Min", "Trackbars", &smin, 255);
		createTrackbar("Sat Max", "Trackbars", &smax, 255);
		createTrackbar("Val Min", "Trackbars", &vmin, 255);
		createTrackbar("Val Max", "Trackbars", &vmax, 255);

		cvtColor(img, imgHSV, COLOR_BGR2HSV);

		Scalar lower(hmin, smin, vmin);
		Scalar upper(hmax, smax, vmax);

		inRange(imgHSV, lower, upper, mask);
		// hmin, smin, vmin, hmax, smax, vmax;
		cout << hmin << ", " << smin << ", " << vmin << ", " << hmax << ", " << smax << ", " << vmax << endl;
		imshow("Image", img);
		imshow("Mask", mask);
		waitKey(1);
	}

		return 0;
}
#endif