#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>

using namespace std;
using namespace cv;

Mat Q;
Mat imgL, imgR;
int disp;
int m = 0;
int n = 0;
double f;
double Tx;
double Cx;
double X, Y, Z;
double a, b, c, d;
vector<Point2f> match_point;
//fstream fs_xyz("points.txt", fstream::out);

//获得屏幕四个角点坐标
void getScrVerPoint(Mat& img, Point2f (&sortCrossPointL)[4]);
double getXYZ(double xyz, double Tx, double Cx, int disp);
void get_panel(Point3d p1, Point3d p2, Point3d p3, double& a, double& b, double& c, double& d);
double dis_pt2panel(Point3d pt, double a, double b, double c, double d);
void on_mouse(int event, int x, int y, int flags, void* param );
int main()
{
	Point2f cornerPointL[4];
	Point2f cornerPointR[4];
	Point3d p[4];
	double distance;
	imgL = imread("3_1.bmp", 0);
	imgR = imread("3_2.bmp", 0);
	if (imgL.empty() || imgR.empty())
	{
		return -1;
	}
	
	getScrVerPoint(imgL, cornerPointL);
	getScrVerPoint(imgR, cornerPointR);

	FileStorage Q_fs("Q.yml", FileStorage::READ);
	Q_fs["Q"] >> Q;
	Q_fs.release();
	f = Q.at<double>(2, 3);
	Tx = -(1 / Q.at<double>(3, 2));
	Cx = Q.at<double>(3, 3) * Tx;
	

	for (size_t i = 0; i < 4 ; i++)
	{
		disp = cornerPointL[i].x - cornerPointR[i].x;
		p[i].x = getXYZ(cornerPointL[i].x + Q.at<double>(0, 3), Tx, Cx, disp);
		p[i].y = getXYZ(cornerPointL[i].y + Q.at<double>(1, 3), Tx, Cx, disp);
		p[i].z = getXYZ(f, Tx, Cx, disp);
		//fs_xyz << X << ',' << Y << ',' << Z << endl;
	}
	get_panel(p[1], p[2], p[3], a, b, c, d);
	distance = dis_pt2panel(p[0], a, b, c, d);
	cout << distance << endl;

	namedWindow("left", 0); 
	namedWindow("right", 0); 
	imshow("left", imgL);
	imshow("right", imgR);
	setMouseCallback("left", on_mouse , (void*) &imgL);
	setMouseCallback("right", on_mouse, (void*) &imgR);

	//fs_xyz.close();
	waitKey(0);
	return 0;
}

void on_mouse(int event, int x, int y, int flags, void* param)
{
	Mat& img = *(Mat*)param;
	Point2d p;
	Point3d pt;
	double dis2plane;
	if (event == EVENT_LBUTTONDOWN)
	{
		p.x = x;
		p.y = y;
		match_point.push_back(p);
		string P = to_string(p.x) + "," + to_string(p.y);
		cout << p.x << "," << p.y << endl;
		//putText(img, P, p, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
		circle(img, p, 5, Scalar(255), -1);

		imshow("left", imgL);
		imshow("right", imgR);
		m++;
	}
	if ((match_point.size() % 2 == 0) && m == 2)
	{
		disp = match_point[0 + n * 2].x - match_point[1 + n * 2].x;
		pt.x = getXYZ(match_point[n * 2].x + Q.at<double>(0, 3), Tx, Cx, disp);
		pt.y = getXYZ(match_point[n * 2].y + Q.at<double>(1, 3), Tx, Cx, disp);
		pt.z = getXYZ(f, Tx, Cx, disp);
		dis2plane = dis_pt2panel(pt, a, b, c, d);
		cout << "disp:" << disp << endl;
		cout << "dis2plane" << dis2plane << endl;
		cout << "distance:" << pt.z << endl;
		cout << "---------------------" << endl;
		n++;
		m -= 2;
	}

}


void get_panel(Point3d p1, Point3d p2, Point3d p3, double& a, double& b, double& c, double& d)
{
	a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
	b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
	c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
	d = 0 - (a * p1.x + b * p1.y + c * p1.z);
}

double dis_pt2panel(Point3d pt, double a, double b, double c, double d)
{
	return abs(a * pt.x + b * pt.y + c * pt.z + d) / sqrt(a * a + b * b + c * c);

}

double getXYZ(double xyz, double Tx, double Cx, int disp)
{
	double Points;
	Points = -xyz * Tx / (disp - Cx);
	return Points;
}

void getScrVerPoint(Mat& img, Point2f(&sortCrossPoint)[4])
{
	Mat srcimg;
	Mat threimg;
	Mat drawimg;
	vector<vector<Point>>contours;
	vector<vector<Point>>contoursR;
	vector<Point2f>approx;
	vector<Point2f>crossPoint;
	srcimg = img;
	threshold(srcimg, threimg, 10, 255, 1);
	drawimg = Mat::zeros(srcimg.size(), srcimg.type());
	findContours(threimg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for (size_t i = 0; i < contours.size(); i++)
	{
		double Area = contourArea(contours[i]);
		if (Area > 300000)
		{
			Point p;
			Moments pp = moments(contours[i], false);
			p.x = pp.m10 / pp.m00;
			p.y = pp.m01 / pp.m00;
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);//多边形逼近
			drawContours(drawimg, contours, i, Scalar(255), 1);
			for (size_t i = 0; i < approx.size(); i++)
			{
				circle(drawimg, approx[i], 300, Scalar(0), -1);
			}
			vector<vector<Point>>contours1;
			vector<double>k_line;
			vector<double>b_line;
			findContours(drawimg, contours1, RETR_EXTERNAL, CHAIN_APPROX_NONE);

			for (size_t i = 0; i < contours1.size(); i++)//获取直线的k、b
			{
				Vec4f lines;
				fitLine(contours1[i], lines, DIST_L1, 0, 0.01, 0.01);
				double k = lines[1] / lines[0];
				double b = lines[3] - k * lines[2];
				Point2f p1, p2;
				p1.x = 0;
				p1.y = b;
				p2.x = drawimg.cols;
				p2.y = k * p2.x + b;
				line(drawimg, p1, p2, Scalar(255), 2);
				k_line.push_back(k);
				b_line.push_back(b);
			}

			for (size_t i = 0; i < k_line.size(); i++)//获取直线交点
			{
				Point2f p;
				p.x = (b_line[(i + 1) % 4] - b_line[i]) / (k_line[i] - k_line[(i + 1) % 4]);
				p.y = k_line[i] * p.x + b_line[i];
				if (p.x > 0 && p.x < drawimg.cols && p.y>0 && p.y < drawimg.rows)
				{
					circle(drawimg, p, 30, Scalar(255), -1);
					crossPoint.push_back(p);
				}
				p.x = (b_line[(i + 2) % 4] - b_line[i]) / (k_line[i] - k_line[(i + 2) % 4]);
				p.y = k_line[i] * p.x + b_line[i];
				if (p.x > 0 && p.x < drawimg.cols && p.y>0 && p.y < drawimg.rows)
				{
					circle(drawimg, p, 30, Scalar(255), -1);
					crossPoint.push_back(p);
				}
				p.x = (b_line[(i + 3) % 4] - b_line[i]) / (k_line[i] - k_line[(i + 3) % 4]);
				p.y = k_line[i] * p.x + b_line[i];
				if (p.x > 0 && p.x < drawimg.cols && p.y>0 && p.y < drawimg.rows)
				{
					circle(drawimg, p, 30, Scalar(255), -1);
					crossPoint.push_back(p);
				}
			}

			for (size_t i = 0; i < crossPoint.size(); i++)//保留四个角点坐标
			{
				for (size_t j = i + 1; j < crossPoint.size(); j++)
				{
					if (crossPoint[i].x == crossPoint[j].x)
					{
						crossPoint.erase(crossPoint.begin() + j);
					}
				}
			}

			for (size_t i = 0; i < crossPoint.size(); i++)//四个角点排序
			{
				if (crossPoint[i].x < p.x && crossPoint[i].y < p.y)
				{
					sortCrossPoint[0] = crossPoint[i];
				}
				else if (crossPoint[i].x > p.x && crossPoint[i].y < p.y)
				{
					sortCrossPoint[1] = crossPoint[i];
				}
				else if (crossPoint[i].x > p.x && crossPoint[i].y > p.y)
				{
					sortCrossPoint[2] = crossPoint[i];
				}
				else if (crossPoint[i].x < p.x && crossPoint[i].y > p.y)
				{
					sortCrossPoint[3] = crossPoint[i];
				}
			}
		}
	}
}