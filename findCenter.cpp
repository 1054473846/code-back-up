#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include<iostream>

using namespace cv;
using namespace std;

//static double angle(Point pt1, Point pt2, Point pt0)
//{
//	double dx1 = pt1.x - pt0.x;
//	double dy1 = pt1.y - pt0.y;
//	double dx2 = pt2.x - pt0.x;
//	double dy2 = pt2.y - pt0.y;
//	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
//}

Point2f getCrossPoint(Point P0, Point P2, Point P1, Point P3)
{
	double ka, kb;

	ka = (double)(P2.y - P0.y) / (double)(P2.x - P0.x); //求出LineA斜率
	kb = (double)(P3.y - P1.y) / (double)(P3.x - P1.x); //求出LineB斜率

	Point2f crossPoint;
	crossPoint.x = (ka * P0.x - P0.y - kb * P1.x + P1.y) / (ka - kb);
	crossPoint.y = (ka * kb * (P0.x - P1.x) + ka * P1.y - kb * P0.y) / (ka - kb);
	return crossPoint;
}

int main()
{

	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 6464);
	cap.set(CAP_PROP_FRAME_HEIGHT, 4852);
	if (!cap.isOpened())
	{
		return -2;
	}
	Mat binaryzation, gray, thresholdImg, deaden, edge, frame;
	Mat resizeFrame;
	Mat aoiFrame;
	Mat rect1, rect2;
	Mat midImg;
	vector<vector<Point>> mid_contours;
	vector<Vec4i> mid_hierarchy;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<Point> approx;
	vector<vector<Point>> squares;
	Point2f Center;
	Point2f imgCenter;
	double lineLength;
	int n = 0;
	namedWindow("frameCenterCircle", WINDOW_NORMAL);
	namedWindow("centerCircle", WINDOW_NORMAL);
	namedWindow("camera", WINDOW_NORMAL);
	namedWindow("threshold", WINDOW_NORMAL);
	while (1)
	{
		cap >> frame;
		aoiFrame = frame(Rect(frame.cols / 5, frame.rows / 5, frame.cols / 5 * 3, frame.rows / 5 * 3));
		imgCenter = Point(aoiFrame.cols / 2, aoiFrame.rows / 2);
		cvtColor(aoiFrame, gray, COLOR_BGR2GRAY);
		midImg = Mat::zeros(frame.size(), CV_8UC1);
		threshold(gray, binaryzation, 80, 255, 1);
		imshow("threshold", binaryzation);
		
		findContours(binaryzation, mid_contours, mid_hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
		for (size_t i = 0; i < mid_contours.size(); i++)
		{
			if (contourArea(mid_contours[i]) > 8000)
			{
				drawContours(midImg, mid_contours, i, Scalar(255), 30, 8, mid_hierarchy);
			}
		}
		findContours(midImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
			if (approx.size() == 4 && contourArea(contours[i]) > 1000 && isContourConvex(Mat(approx)))
			{
				Point2f PointCenter = getCrossPoint(approx[0], approx[2], approx[1], approx[3]);
				if (PointCenter.x > (aoiFrame.cols / 8) && PointCenter.x < (aoiFrame.cols / 8 * 7))
				{
					for (size_t j = 0; j < 4; j++)
					{
						line(aoiFrame, approx[j], approx[(j + 1) % 4], Scalar(0, 0, 255), 2);
					}
					double rectWidth = sqrtf(powf((approx[0].y - approx[1].y), 2) + powf((approx[0].x - approx[1].x), 2));
					double rectHeight = sqrtf(powf((approx[2].y - approx[1].y), 2) + powf((approx[2].x - approx[1].x), 2));
					double rectX = PointCenter.x - rectWidth / 40;
					double rectY = PointCenter.y - rectWidth / 40;
					rect1 = thresholdImg(Rect(rectX, rectY, rectWidth / 18, rectWidth / 18));
					rect2 = aoiFrame(Rect(rectX, rectY, rectWidth / 18, rectWidth / 18));
					Moments m = moments(rect1, true);
					Center.x = m.m10 / m.m00 + rectX;
					Center.y = m.m01 / m.m00 + rectY;
					if (rect1.cols > 3)
					{
						line(aoiFrame, Center, imgCenter, Scalar(255, 255, 255), 2);
						imshow("centerCircle", rect1);
						imshow("frameCenterCircle", rect2);
					}
				}
				lineLength = sqrtf(powf((Center.x - imgCenter.x), 2) + powf((Center.y - imgCenter.y), 2));
				if (n % 10 == 0)
				{
					cout << "distance: " << lineLength << " pixels" << endl;
				}
			}
		}
		imshow("camera", frame);
		n++;
		waitKey(30);
	}
	return 0;
}

