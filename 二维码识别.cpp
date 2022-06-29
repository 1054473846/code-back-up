#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;
#define VVP vector<vector<Point>>
#define VP vector<Point>
int main()
{
	Mat srcimg = imread("testImg\\csdn¶þÎ¬Âë.png", 1);
	if (srcimg.empty())
	{
		return -1;
	}
	Mat grayimg = srcimg;
	if (srcimg.channels() == 3)
	{
		cvtColor(srcimg, grayimg, COLOR_BGR2GRAY);
	}
	Mat threshimg;
	threshold(grayimg, threshimg, 5, 255, 1);
	VVP contours;
	vector<Point2f> ctPoint;
	vector<Vec4i> hierarchy;
	Mat drawimg = Mat::zeros(srcimg.size(), CV_8U);
	findContours(threshimg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++)
	{
		if (hierarchy[i][2] > 0 && hierarchy[i][3] > 0 && contourArea(contours[i]) > 225 && contourArea(contours[i]) < 450)
		{
			Moments m = moments(contours[i]);
			Point pt;
			pt.x = m.m10 / m.m00;
			pt.y = m.m01 / m.m00;
			ctPoint.push_back(pt);
			drawContours(drawimg, contours, i, Scalar(255), -1);
		}
	}
	Point2f warpPoint[3];
	int maxSite = 0;
	float maxY = 0.0;
	for (int i = 0; i < ctPoint.size(); i++)
	{
		if (ctPoint[i].y > maxY)
		{
			maxY = ctPoint[i].y;
			maxSite = i;
		}
	}
	warpPoint[0] = Point2f(ctPoint[maxSite].x, ctPoint[maxSite].y);
	ctPoint.erase(ctPoint.begin() + maxSite);
	if (ctPoint[0].x < ctPoint[1].x)
	{
		warpPoint[1] = Point2f(ctPoint[0].x, ctPoint[maxSite].y);
		warpPoint[2] = Point2f(ctPoint[1].x, ctPoint[maxSite].y);
	}
	else
	{
		warpPoint[1] = Point2f(ctPoint[1].x, ctPoint[maxSite].y);
		warpPoint[2] = Point2f(ctPoint[0].x, ctPoint[maxSite].y);
	}
	Mat warpimg = Mat::zeros(Size(400, 400), CV_8U);
	Point2f afterWarpPoint[3];
	afterWarpPoint[0] = Point(50, 350);
	afterWarpPoint[1] = Point(50, 50);
	afterWarpPoint[2] = Point(350, 50);
	Mat m = getAffineTransform(warpPoint, afterWarpPoint);
	warpAffine(grayimg, warpimg, m, warpimg.size());
	waitKey(1);
	return 0;
}