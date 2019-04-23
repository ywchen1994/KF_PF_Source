#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include"time.h"
using namespace cv;
using namespace std;

const int winHeight = 600;
const int winWidth = 800;
struct M
{
	uint16_t a;

};

Point mousePosition = Point(winWidth >> 1, winHeight >> 1);

//mouse event callback
void mouseEvent(int event, int x, int y, int flags, void *param)
{
	if (event == CV_EVENT_MOUSEMOVE) {
		mousePosition = Point(x, y);
	}
}
Point2d oldPoint;
int main(void)
{
	RNG rng;

	const int stateNum = 4;
	const int measureNum = 4;
	KalmanFilter KF(stateNum, measureNum, 0);

	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(1));
	KF.statePost= *(Mat_<float>(4,1) << 400,300,0,0);
	//rng.fill(KF.statePost, RNG::UNIFORM, 0, winHeight > winWidth ? winWidth : winHeight);
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
	oldPoint = Point2d(KF.statePost.at<float>(0), KF.statePost.at<float>(1));
	namedWindow("kalman");
	setMouseCallback("kalman", mouseEvent);

	Mat image(winHeight, winWidth, CV_8UC3, Scalar(0));
	time_t t1 = clock();
	while (1)
	{
		time_t t2 = clock();
		//2.kalman prediction
		Mat prediction = KF.predict();
		Point predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));
		float time = (float)(t2 - t1) / CLK_TCK;
		//3.update measurement
		measurement.at<float>(0) = (float)mousePosition.x;
		measurement.at<float>(1) = (float)mousePosition.y;
	
		measurement.at<float>(2) = (float)(oldPoint.x- mousePosition.x)/ (float)time;
		measurement.at<float>(3) = (float)(oldPoint.y - mousePosition.y) / (float)time;
		if (time == 0)
		{
			measurement.at<float>(2) = 0;
			measurement.at<float>(3) = 0;
		}
		double v = sqrt(pow(measurement.at<float>(2), 2) + pow(measurement.at<float>(3), 2));
		//4.update
		KF.correct(measurement);

		//draw 
		image.setTo(Scalar(255, 255, 255, 0));
		circle(image, predict_pt, 5, Scalar(0, 255, 0), 3);    //predicted point with green
		circle(image, mousePosition, 5, Scalar(255, 0, 0), 3); //current position with red		

		char buf[256];
		if (v != 0)
			int a = 99;
		sprintf_s(buf, 256, "predicted position:(%3d,%3d,%3d)", predict_pt.x, predict_pt.y, v);
		putText(image, buf, Point(10, 30), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);
		sprintf_s(buf, 256, "current position :(%3d,%3d)", mousePosition.x, mousePosition.y);
		putText(image, buf, cvPoint(10, 60), CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);
	
		imshow("kalman", image);
		int key = waitKey(3);
		if (key == 27) {//esc   
			break;
		}
		t1 = t2;
		oldPoint = Point2d(mousePosition.x, mousePosition.y);
	}

}