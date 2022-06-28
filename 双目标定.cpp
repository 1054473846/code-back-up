#include<iostream>
#include<opencv2/opencv.hpp>
#include<Windows.h>
#include <fstream> 

using namespace std;
using namespace cv;

Mat R, T, E, F;//R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵  
Mat cameraMatrixL;//左右相机内参
Mat distCoeffL;
Mat cameraMatrixR;
Mat distCoeffR;
Mat Q;
double f;
double Tx;
double Cx;
double X, Y, Z;
Point2f getCrossPoint(Point P0, Point P2, Point P1, Point P3);
Point2f getCornerPoint(Mat& srcImg, Mat& threshImg, Point2f(&srcTraPoint)[4]);
vector<vector<Point2f>> getALLCirclePoint(Mat srcImg, Mat threshImg, int minAreas, int maxAreas, String& fileName,  Point2f boardCenter);
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], vector<vector<Point2f>> circlePoint);
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], int minAreas, int maxAreas, String& fileName);
void getRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, double squaresize);
void outputCameraParam();
double getDistanceZ(double f, double Tx, double Cx, int disp);
double getDistanceX(double y, double Tx, double Cx, int disp);
double getDistanceY(double y, double Tx, double Cx, int disp);
int stime = 0;
float etime = 0;
int main(int argc, char* argv[])
{
	String address = "E:\\公司文件\\2022-02-16标定\\left";
	String addressR = "E:\\公司文件\\2022-02-16标定\\right";
	vector<String> fn;
	vector<String> fnR;
	vector<vector<Point2f>> picCirclePointL;
	vector<vector<Point2f>> picCirclePointR;
	vector<vector<Point3f>> objRealPoint;
	int imgWidth = 0;
	int imgHeight = 0;
	double disp = 0;
	glob(address, fn, false);
	glob(addressR, fnR, false);
	int fn_size = fn.size();
	fstream fws("point2.txt", fstream::out);
	for (size_t i = 0; i < fn_size; i++)
	{
		Mat imgL = imread(fn[i], 0);
		Mat imgR = imread(fnR[i], 0);
		continue;

		int start_time = GetTickCount64();
		/*********left***********/
		String name_imgL = fn[i];
		int add_length = name_imgL.find_last_of('\\') + 1;
		string fileName = name_imgL.substr(add_length, name_imgL.length() - add_length);//substr:截取字符串长度
		Mat src_img, thresh_Img;
		Point2f src_tra_pointL[4];
		Point2f board_centerL;
		vector<Point2f> mid_circleL;
		src_img = imread(name_imgL, 0);
		imgWidth = src_img.cols;
		imgHeight = src_img.rows;
		if (src_img.empty())
		{
			return -1;
		}
		threshold(src_img, thresh_Img, 70, 255, 1);
		board_centerL = getCornerPoint(src_img, thresh_Img, src_tra_pointL);
		//mid_circleL = getSortPoint(src_img, src_tra_point, 10, 8000, fileName);
		vector<vector<Point2f>> circle_pointL = getALLCirclePoint(src_img, thresh_Img, 100, 10000, fileName, board_centerL);
		mid_circleL = getSortPoint(src_img, src_tra_pointL, circle_pointL);
		picCirclePointL.push_back(mid_circleL);

		/*********right***********/
		String name_imgR = fnR[i];
		int add_lengthR = name_imgR.find_last_of('\\') + 1;
		string fileNameR = name_imgR.substr(add_lengthR, name_imgR.length() - add_lengthR);//substr:截取字符串长度
		Mat src_imgR, thresh_ImgR;
		Point2f src_tra_pointR[4];
		Point2f board_centerR;
		vector<Point2f> mid_circleR;
		src_imgR = imread(name_imgR, 0);
		if (src_imgR.empty())
		{
			return -1;
		}
		threshold(src_imgR, thresh_ImgR, 80, 255, 1);
		board_centerR = getCornerPoint(src_imgR, thresh_ImgR, src_tra_pointR);
		mid_circleR = getSortPoint(src_imgR, src_tra_pointR, 10, 8000, fileNameR);
		//vector<vector<Point2f>> circle_pointR = getALLCirclePoint(src_imgR, thresh_ImgR, 100, 10000, fileNameR, board_centerR);
		//mid_circleR = getSortPoint(src_imgR, src_tra_pointR, circle_pointR);
		picCirclePointR.push_back(mid_circleR);
		float end_time = (GetTickCount64() - start_time) / 1000.0;
		cout << fileName << " + " << fileNameR << " cost time:" << end_time << endl;
		cout << "********************************" << endl;



		//FileStorage Q_fs("Q.yml", FileStorage::READ);
		//Q_fs["Q"] >> Q;
		//Q_fs.release();
		//f = Q.at<double>(2, 3);
		//Tx = -(1 / Q.at<double>(3, 2));
		//Cx = Q.at<double>(3, 3) * Tx;

		//for (size_t j = 0; j < mid_circleL.size(); j++)
		//{
		//	disp = (double)mid_circleL[j].x - (double)mid_circleR[j].x;
		//	X = (getDistanceX(mid_circleL[j].x + Q.at<double>(0, 3), Tx, Cx, disp)) + (i % 4) * 100;
		//	Y = getDistanceY(mid_circleL[j].y + Q.at<double>(1, 3), Tx, Cx, disp) + (i / 4) * 100;
		//	Z = getDistanceZ(f, Tx, Cx, disp);
		//	fws << X << ',' << Y << ',' << Z << endl;
		//	string dis = to_string(Z);
		//	putText(src_img, dis, Point(mid_circleL[j].x + 20, mid_circleL[j].y + 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);
		//}
		waitKey(30);

	}

	FileStorage cameraMatrixL_fs("left_intrinsic.xml", cv::FileStorage::READ);
	cameraMatrixL_fs["intrinsic"] >> cameraMatrixL;//指的是.xml的id
	cameraMatrixL_fs["distortion_coeff"] >> distCoeffL;//指的是.xml的id
	FileStorage cameraMatrixR_fs("right_intrinsic.xml", cv::FileStorage::READ);
	cameraMatrixR_fs["intrinsic"] >> cameraMatrixR;//指的是.xml的id
	cameraMatrixR_fs["distortion_coeff"] >> distCoeffR;//指的是.xml的id

	getRealPoint(objRealPoint, 15, 15, fn_size, 3.6);
	double rms = stereoCalibrate(objRealPoint, picCirclePointL, picCirclePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		Size(imgWidth, imgHeight), R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << rms << endl;
	outputCameraParam();
	return 0;
}
//2021-12-13 rms:0.178
//2021-12-13 rms:0.199
//2021-12-20 rms:0.242
//2021-12-23 rms:0.217
//2022-01-12 rms:0.24


double getDistanceZ(double f, double Tx, double Cx, int disp)
{
	double distance;
	distance = -f * Tx / (disp - Cx);
	return distance;
}

double getDistanceX(double x, double Tx, double Cx, int disp)
{
	double X;
	X = -x * Tx / (disp - Cx);
	return X;
}

double getDistanceY(double y, double Tx, double Cx, int disp)
{
	double Y;
	Y = -y * Tx / (disp - Cx);
	return Y;
}

//保存标定结果
void outputCameraParam(void)
{
	FileStorage fs("binocular_intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "cameraMatrixL" << cameraMatrixL
			<< "cameraDistcoeffL" << distCoeffL
			<< "cameraMatrixR" << cameraMatrixR
			<< "cameraDistcoeffR" << distCoeffR;
		fs.release();
	}
	else
		cout << "can not save the intrinsics。。。" << endl;

	fs.open("binocular_extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T;
		fs.release();
	}
	else
		cout << "can not save the extrinsic parameters\n";
}

//获得设定的世界坐标
void getRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, double squaresize)
{
	vector<Point3f> imgpoint;
	for (int i = 0; i < (boardheight / 5); i++)
	{
		for (int j = 0; j < boardwidth; j++)
		{
			imgpoint.push_back(Point3f(i * 7 * squaresize, j * squaresize, 0));
		}
	}

	for (int i = 0; i < (boardwidth / 5); i++)
	{
		for (int j = 0; j < boardheight; j++)
		{
			imgpoint.push_back(Point3f(j * squaresize, i * 7 * squaresize, 0));
		}
	}

	for (int i = 0; i < imgNumber; i++)
	{
		obj.push_back(imgpoint);
	}
}

//获得所有排序后的角点坐标(投影变换)
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], int minAreas, int maxAreas, String& fileName)
{
	Mat m;
	Mat back_m;
	Mat srcClone = srcImg.clone();
	Mat dstTransform(Size(800, 800), srcImg.type());
	Point2f dstTraPoint[4];
	vector<Point2f> one_circle_point;
	dstTraPoint[0] = Point(0, 0);
	dstTraPoint[1] = Point(dstTransform.cols, 0);
	dstTraPoint[2] = Point(dstTransform.cols, dstTransform.rows);
	dstTraPoint[3] = Point(0, dstTransform.rows);
	m = getPerspectiveTransform(srcTraPoint, dstTraPoint);
	back_m = getPerspectiveTransform(dstTraPoint, srcTraPoint);
	warpPerspective(srcImg, dstTransform, m, dstTransform.size());
	SimpleBlobDetector::Params params;
	params.filterByArea = true;//使用面积筛选
	params.filterByCircularity = false;//使用圆度筛选
	if (params.filterByArea)
	{
		params.maxArea = maxAreas;
		params.minArea = minAreas;
	}
	if (params.filterByCircularity)
	{
		params.minCircularity = 0.25;
		params.maxCircularity = 1;
	}
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	vector<Point2f> corner_circle_centers;
	vector<Point2f> circle_centers;
	vector<vector<Point2f>> points;
	for (size_t i = 0; i < 6; i++)
	{
		int  height = i * 7 > 15 ? 15 : 1;
		int width = i * 7 < 15 ? 15 : 1;
		Size cornSize(width, height);
		bool found = findCirclesGrid(dstTransform, cornSize, circle_centers, CALIB_CB_SYMMETRIC_GRID, blobDetector);//寻找圆心
		if (found)
		{
			points.push_back(circle_centers);
			drawChessboardCorners(srcImg, cornSize, circle_centers, found);
			circle(dstTransform, circle_centers[3], 50, Scalar(255), -1);
			cout << fileName << "寻找角点成功 " << i + 1 << " 次" << endl;
		}
	}



	for (size_t i = 0; i < points.size() * 3; i++)//设定的X方向角点
	{
		int n = one_circle_point.size() / 15;
		if (abs((points[i % 6][14].x - points[i % 6][0].x)) < 50 && n < 3)
		{
			if (points[i % 6][0].x < ((n + 1) * 300) && points[i % 6][0].x >(n * 300))
			{
				if (points[i % 6][0].y < 400)
				{
					for (size_t j = 0; j < points[i % 6].size(); j++)
					{
						one_circle_point.push_back(points[i % 6][j]);
					}
				}
				else
				{
					for (size_t j = 0; j < points[i % 6].size(); j++)
					{
						one_circle_point.push_back(points[i % 6][14 - j]);
					}

				}
			}
		}
	}
	for (size_t i = 0; i < points.size() * 3; i++)//设定的Y方向角点
	{
		int n = one_circle_point.size() / 15 - 3;
		if (abs((points[i % 6][14].y - points[i % 6][0].y)) < 50)
		{
			if (points[i % 6][0].y < ((n + 1) * 300) && points[i % 6][0].y >(n * 300))
			{
				if (points[i % 6][0].x < 400)
				{
					for (size_t j = 0; j < points[i % 6].size(); j++)
					{
						one_circle_point.push_back(points[i % 6][j]);
					}
				}
				else
				{
					for (size_t j = 0; j < points[i % 6].size(); j++)
					{
						one_circle_point.push_back(points[i % 6][14 - j]);
					}
				}
			}
		}
	}
	perspectiveTransform(one_circle_point, one_circle_point, back_m);

	waitKey(30);
	return one_circle_point;

}


//获得所有排序后的角点坐标
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], vector<vector<Point2f>> circlePoint)
{
	Mat m;
	Mat back_m;
	Mat srcClone = srcImg.clone();
	Mat dstTransform(Size(800, 800), srcImg.type());
	Point2f dstTraPoint[4];
	vector<Point2f> one_circle_point;
	dstTraPoint[0] = Point(0, 0);
	dstTraPoint[1] = Point(dstTransform.cols, 0);
	dstTraPoint[2] = Point(dstTransform.cols, dstTransform.rows);
	dstTraPoint[3] = Point(0, dstTransform.rows);
	m = getPerspectiveTransform(srcTraPoint, dstTraPoint);
	back_m = getPerspectiveTransform(dstTraPoint, srcTraPoint);
	warpPerspective(srcImg, dstTransform, m, dstTransform.size());
	for (size_t i = 0; i < circlePoint.size(); i++)
	{
		perspectiveTransform(circlePoint[i], circlePoint[i], m);
	}
	for (size_t i = 0; i < circlePoint.size() * 3; i++)//设定的X方向角点
	{
		int n = one_circle_point.size() / 15;
		if (abs((circlePoint[i % 6][14].x - circlePoint[i % 6][0].x)) < 50 && n < 3)
		{
			if (circlePoint[i % 6][0].x < ((n + 1) * 300) && circlePoint[i % 6][0].x >(n * 300))
			{
				if (circlePoint[i % 6][0].y < 400)
				{
					for (size_t j = 0; j < circlePoint[i % 6].size(); j++)
					{
						one_circle_point.push_back(circlePoint[i % 6][j]);
					}
				}
				else
				{
					for (size_t j = 0; j < circlePoint[i % 6].size(); j++)
					{
						one_circle_point.push_back(circlePoint[i % 6][14 - j]);
					}

				}
			}
		}
	}
	for (size_t i = 0; i < circlePoint.size() * 3; i++)//设定的Y方向角点
	{
		int n = one_circle_point.size() / 15 - 3;
		if (abs((circlePoint[i % 6][14].y - circlePoint[i % 6][0].y)) < 50)
		{
			if (circlePoint[i % 6][0].y < ((n + 1) * 300) && circlePoint[i % 6][0].y >(n * 300))
			{
				if (circlePoint[i % 6][0].x < 400)
				{
					for (size_t j = 0; j < circlePoint[i % 6].size(); j++)
					{
						one_circle_point.push_back(circlePoint[i % 6][j]);
					}
				}
				else
				{
					for (size_t j = 0; j < circlePoint[i % 6].size(); j++)
					{
						one_circle_point.push_back(circlePoint[i % 6][14 - j]);
					}
				}
			}
		}
	}
	perspectiveTransform(one_circle_point, one_circle_point, back_m);
	return one_circle_point;
}

//获得标定板所有圆心坐标
vector<vector<Point2f>> getALLCirclePoint(Mat srcImg, Mat threshImg, int minAreas, int maxAreas, String& fileName, Point2f boardCenter)
{
	stime = GetTickCount64();
	Mat blackImg = Mat::zeros(srcImg.size(), srcImg.type());
	Mat whiteImg = Mat::ones(srcImg.size(), srcImg.type());
	whiteImg = whiteImg * 255;
	Mat bin_img;
	Point2f mid_circle_corner;
	double min = 999999;
	vector<vector<Point> > contours;
	findContours(threshImg, contours, RETR_LIST, CHAIN_APPROX_NONE);// 轮廓检測
	for (int i = 0; i < contours.size(); ++i)
	{
		double areas = contourArea(contours[i]);
		if (areas > minAreas && areas < maxAreas)// （过大或过小）排除轮廓
		{
			Mat pointsf;
			Mat(contours[i]).convertTo(pointsf, CV_32F);
			RotatedRect box = fitEllipse(pointsf);// 椭圆形匹配
			ellipse(blackImg, box, Scalar(255), -1, 8);// 绘制出椭圆
			srcImg.copyTo(whiteImg, blackImg);
		}
	}
	//Mat aoiSrcimg = whiteImg(Rect(boardCenter.x - 1200, boardCenter.y - 1200, 2400, 2400));
	//opencv斑点检测器
	SimpleBlobDetector::Params params;
	params.filterByArea = true;//使用面积筛选
	params.filterByCircularity = false;//使用圆度筛选
	if (params.filterByArea)
	{
		params.maxArea = maxAreas;
		params.minArea = minAreas;
	}
	if (params.filterByCircularity)
	{
		params.minCircularity = 0.25;
		params.maxCircularity = 1;
	}
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	vector<Point2f> corner_circle_centers;
	vector<Point2f> circle_centers;
	vector<vector<Point2f>> points;
	for (size_t i = 0; i < 6; i++)
	{
		int  height = i * 7 > 15 ? 15 : 1;
		int width = i * 7 < 15 ? 15 : 1;
		Size cornSize(width, height);
		bool found = findCirclesGrid(whiteImg, cornSize, circle_centers, CALIB_CB_SYMMETRIC_GRID, blobDetector);//寻找圆心
		if (found)
		{
			circle(whiteImg, circle_centers[3], 50, Scalar(255), -1);
			drawChessboardCorners(srcImg, cornSize, circle_centers, found);
			//for (size_t k = 0; k < circle_centers.size(); k++)
			//{
			//	circle_centers[k].x += boardCenter.x - 1200;
			//	circle_centers[k].y += boardCenter.y - 1200;
			//}
			points.push_back(circle_centers);
			if (points.size() > 5)
			{
				cout << fileName << "寻找角点成功 " << endl;
			}
		}
	}
	etime = (GetTickCount64() - stime) / 1000.0;
	//cout << fileName << " find circle point cost time: " << etime << " s" << endl;
	return points;
}

//获得标定板方向
Point2f getCornerPoint(Mat& srcImg, Mat& threshImg, Point2f(&srcTraPoint)[4])
{
	Mat corner_aoi_img;
	Mat midImg = Mat::zeros(srcImg.size(), CV_8UC1);
	Mat dstImg = Mat::zeros(srcImg.size(), CV_8UC1);
	vector<vector<Point>> mid_contours;
	vector<vector<Point>> contours;
	vector<Vec4i> mid_hierarchy;
	vector<Vec4i> hierarchy;
	vector<Point> approx;
	Point2f PointCenter;
	findContours(threshImg, mid_contours, mid_hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < mid_contours.size(); i++)
	{
		if (contourArea(mid_contours[i]) > 8000)
		{
			drawContours(midImg, mid_contours, i, Scalar(255), 100, 8, mid_hierarchy);
		}
	}
	findContours(midImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
		if (approx.size() == 4 && contourArea(contours[i]) > 3000 && isContourConvex(Mat(approx)) && hierarchy[i][3] != -1)
		{
			PointCenter = getCrossPoint(approx[0], approx[2], approx[1], approx[3]);
			if (PointCenter.x > (srcImg.cols / 8) && PointCenter.x < (srcImg.cols / 8 * 7))
			{
				int min = 999999;
				for (size_t n = 0; n < approx.size(); n++)
				{
					int a = 0;
					corner_aoi_img = threshImg(Rect(approx[n].x - 20, approx[n].y - 20, 40, 40));
					for (size_t i = 0; i < corner_aoi_img.rows; i++)
					{
						uchar* p = corner_aoi_img.ptr<uchar>(i);
						for (size_t j = 0; j < corner_aoi_img.cols; j++)
						{
							a += p[j];
						}
					}
					if (a < min)
					{
						min = a;
						srcTraPoint[0] = approx[n];
						srcTraPoint[1] = approx[(n + 1) % 4];
						srcTraPoint[2] = approx[(n + 2) % 4];
						srcTraPoint[3] = approx[(n + 3) % 4];
					}
				}
			}
		}
	}
	return PointCenter;
}

//获得标定板中心点近似坐标
Point2f getCrossPoint(Point P0, Point P2, Point P1, Point P3)
{
	double ka, kb;
	Point2f crossPoint;
	ka = ((double)P2.y - (double)P0.y) / ((double)P2.x - (double)P0.x); //求出LineA斜率
	kb = ((double)P3.y - (double)P1.y) / ((double)P3.x - (double)P1.x); //求出LineB斜率
	crossPoint.x = (ka * P0.x - P0.y - kb * P1.x + P1.y) / (ka - kb);
	crossPoint.y = (ka * kb * ((double)P0.x - (double)P1.x) + ka * P1.y - kb * P0.y) / (ka - kb);
	return crossPoint;
}