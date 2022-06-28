#include<iostream>
#include<opencv2/opencv.hpp>
#include<Windows.h>

using namespace std;
using namespace cv;

Point2f getCrossPoint(Point P0, Point P2, Point P1, Point P3);
Point2f getRectCornerPoint(Mat& srcImg, Mat& threshImg, Point2f(&srcTraPoint)[4]);
vector<vector<Point2f>> getALLCirclePoint(Mat srcImg, Mat& threshImg, int minAreas, int maxAreas, String& fileName, Point2f boardCenter);
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], vector<vector<Point2f>> circlePoint);
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], int minAreas, int maxAreas, String& fileName);
void getRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, double squaresize);
int timeStart = 0;
float timeEnd = 0;
//1-left  2-right  3-both
#define mm 2

#define nn 1
int main(int argc, char* argv[])
{
	//左相机标定
	if (mm == 1 || mm == 3)
	{
		String address = "E:\\公司文件\\2022-02-16标定\\left";
		vector<String> fn;
		glob(address, fn, false);
		vector<vector<Point2f>> picCirclePoint;
		vector<vector<Point3f>> objRealPoint;
		Mat intrinsic;        //相机内参数  
		Mat distortion_coeff; //相机畸变参数  
		vector<Mat> rvecs;    //旋转向量  
		vector<Mat> tvecs;    //平移向量  
		int imgWidth = 0;
		int imgHeight = 0;
		
		for (size_t i = 0; i < fn.size(); i++)
		{
			int stime = GetTickCount64();
			String name_img = fn[i];
			int add_length = name_img.find_last_of('\\') + 1;//substr(pos,n):返回一个string，包含s中从pos开始的n个字符的拷贝。
			string fileName = name_img.substr(add_length, name_img.length() - add_length);//substr:截取字符串长度
			Mat src_img, thresh_Img;
			Point2f rect_point[4];
			Point2f board_center;
			vector<Point2f> sort_circle_point;
			vector<vector<Point2f>> circle_point;
			src_img = imread(name_img, 0);
			imgWidth = src_img.cols;
			imgHeight = src_img.rows;
			if (src_img.empty())
			{
				return -1;
			}
			threshold(src_img, thresh_Img, 70, 255, 1);
			board_center = getRectCornerPoint(src_img, thresh_Img, rect_point);
			circle_point = getALLCirclePoint(src_img, thresh_Img, 10, 10000, fileName, board_center);
			sort_circle_point = getSortPoint(src_img, rect_point, circle_point);
			//sort_circle_point = getSortPoint(src_img, rect_point, 10, 8000, fileName);
			picCirclePoint.push_back(sort_circle_point);
			float etime = (GetTickCount64() - stime) / 1000.0;
			cout << fileName << " 总花费时间：" << etime << " s" << endl;
			cout << "****************************" << endl;
			waitKey(30);
		}

		getRealPoint(objRealPoint, 15, 15, fn.size(), 3.6);
		calibrateCamera(objRealPoint, picCirclePoint, Size(imgWidth, imgHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
		FileStorage fswrite("left_intrinsic.xml", FileStorage::WRITE);
		fswrite << "intrinsic" << intrinsic;
		fswrite << "distortion_coeff" << distortion_coeff;
		fswrite.release();
	}

	//右相机标定
	if (mm == 2 || mm == 3)
	{
		String address = "E:\\公司文件\\2022-02-16标定\\right";
		vector<String> fn;
		glob(address, fn, false);
		vector<vector<Point2f>> picCirclePoint;
		vector<vector<Point3f>> objRealPoint;
		Mat intrinsic;        //相机内参数  
		Mat distortion_coeff; //相机畸变参数  
		vector<Mat> rvecs;    //旋转向量  
		vector<Mat> tvecs;    //平移向量  
		int imgWidth = 0;
		int imgHeight = 0;
		for (size_t i = 13; i < fn.size(); i++)
		{
			String name_img = fn[i];
			int add_length = name_img.find_last_of('\\') + 1;//substr(pos,n):返回一个string，包含s中从pos开始的n个字符的拷贝。
			string fileName = name_img.substr(add_length, name_img.length() - add_length);//substr:截取字符串长度
			Mat src_img, thresh_Img;
			Point2f rect_point[4];
			Point2f board_center;
			vector<Point2f> sort_circle_point;
			src_img = imread(name_img, 0);
			imgWidth = src_img.cols;
			imgHeight = src_img.rows;
			if (src_img.empty())
			{
				return -1;
			}
			threshold(src_img, thresh_Img, 50, 255, 1);
			board_center = getRectCornerPoint(src_img, thresh_Img, rect_point);
			if (nn == 1)//先检测圆心坐标
			{
				vector<vector<Point2f>> circle_point = getALLCirclePoint(src_img, thresh_Img, 100, 8000, fileName, board_center);
				sort_circle_point = getSortPoint(src_img, rect_point, circle_point);
			}
			else//先透视变换，再获取圆心坐标
			{
				sort_circle_point = getSortPoint(src_img, rect_point, 10, 8000, fileName);
			}
			picCirclePoint.push_back(sort_circle_point);
			cout << "****************************" << endl;
			waitKey(30);
		}

		getRealPoint(objRealPoint, 15, 15, fn.size(), 3.6);
		calibrateCamera(objRealPoint, picCirclePoint, Size(imgWidth, imgHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
		FileStorage fswrite("right_intrinsic.xml", FileStorage::WRITE);
		fswrite << "intrinsic" << intrinsic;
		fswrite << "distortion_coeff" << distortion_coeff;
		fswrite.release();
	}

	return 0;
}

//获得设定的世界坐标
void getRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, double squaresize)
{
	vector<Point3f> imgpoint;
	for (int i = 0; i < (boardheight / 5); i++)
	{
		for (int j = 0; j < boardwidth; j++)
		{
			imgpoint.push_back(Point3f(j * squaresize, i * 7 * squaresize, 0));
		}
	}

	for (int i = 0; i < (boardwidth / 5); i++)
	{
		for (int j = 0; j < boardheight; j++)
		{
			imgpoint.push_back(Point3f(i * 7 * squaresize, j * squaresize, 0));
		}
	}

	for (int i = 0; i < imgNumber; i++)
	{
		obj.push_back(imgpoint);
	}
}

//获得所有排序后的角点坐标(透视变换)
vector<Point2f> getSortPoint(Mat& srcImg, Point2f srcTraPoint[4], int minAreas, int maxAreas, String& fileName)
{
	timeStart = GetTickCount64();
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
			//drawChessboardCorners(srcImg, cornSize, circle_centers, found);
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

	timeEnd = (GetTickCount64() - timeStart) / 1000.0;
	cout << "获得排序后的圆心坐标-花费时间：" << timeEnd << " s" << endl;

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
vector<vector<Point2f>> getALLCirclePoint(Mat srcImg, Mat& threshImg, int minAreas, int maxAreas, String& fileName, Point2f boardCenter)
{
	timeStart = GetTickCount64();
	//Mat aoiSrcimg = srcImg(Rect(boardCenter.x - 1300, boardCenter.y - 1300, 2600, 2600));
	
	Mat blackImg = Mat::zeros(srcImg.size(), srcImg.type());
	Mat whiteImg = Mat::ones(srcImg.size(), srcImg.type());
	whiteImg = whiteImg * 255;
	Mat bin_img;
	Point2f mid_circle_corner;
	double min = 999999;
	vector<vector<Point> > contours;
	//threshold(srcImg, bin_img, 95, 255, 1);// 图像的二值化
	findContours(threshImg, contours, RETR_LIST, CHAIN_APPROX_NONE);// 轮廓检測
	for (int i = 0; i < contours.size(); ++i)
	{
		double areas = contourArea(contours[i]);
		if (areas > minAreas && areas < maxAreas && contours[i][0].x > 300 && contours[i][0].x < 6200)// （过大或过小）排除轮廓
		{
			Mat pointsf;
			Mat(contours[i]).convertTo(pointsf, CV_32F);
			RotatedRect box = fitEllipse(pointsf);// 椭圆形匹配
			ellipse(blackImg, box, Scalar(255), -1, 8);// 绘制出椭圆
		}
	}
	srcImg.copyTo(whiteImg, blackImg);
	//Mat aoiSrcimg = whiteImg(Rect(boardCenter.x - 1650, boardCenter.y - 1650, 3300, 3300));
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
			drawChessboardCorners(srcImg, cornSize, circle_centers, found);
			circle(whiteImg, circle_centers[3], 50, Scalar(255), -1);
			//for (size_t k = 0; k < circle_centers.size(); k++)
			//{
			//	circle_centers[k].x += boardCenter.x - 1650;
			//	circle_centers[k].y += boardCenter.y - 1650;
			//}
			points.push_back(circle_centers);
			if (points.size()>5)
			{
				cout << fileName << "寻找角点成功 " << endl;
			}
		}
	}
	timeEnd = (GetTickCount64() - timeStart)/1000.0;
	cout<<"获得标定板所有圆心坐标-花费时间：" << timeEnd << " s" << endl;
	return points;
}

//获得标定板方向
Point2f getRectCornerPoint(Mat& srcImg, Mat& threshImg, Point2f(&srcTraPoint)[4])
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
		if (contourArea(mid_contours[i]) > 5000)
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