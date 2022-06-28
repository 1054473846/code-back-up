#include<iostream>
#include<opencv2/opencv.hpp>
#include<fstream>
#include<Windows.h>
using namespace std;
using namespace cv;

const int value = 100;

void DefectsRepair(Mat& pBinary, float fDisThre);
bool comparePoint(Point2f a, Point2f b)
{
	return a.x < b.x;
}
void getCenterPoint(Mat& srcImg, vector<Point2f>& centerPt);
float getValue(Mat& inputImg,vector<Point2f> ptM,vector<vector<float>>matchPtLength );
void Log(fstream& out , int i);
void getCtByCanny(Mat& inputImg , vector<Point2f>& centerPt);
void getPixelError(vector<Point2f>& ptM, vector<Point2f>& ptL, vector<vector<float>>& all_corner_distance, fstream& fs, Mat& m, double& pixel_um);

int main()
{
	char Buf[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, Buf);
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	string TimePath = to_string(sys.wYear) + '.' + to_string(sys.wMonth) + '.' + to_string(sys.wDay) + '.';
	string LogPath = string(Buf) + "\\log\\" + TimePath + "--.log";
	fstream fs;
	fs.open(LogPath, ios::out | ios::app);

	Mat m, back_m;
	double pixel_um = 0;
	FileStorage fsread("perspectiveMatrix.yml", FileStorage::READ);
	fsread["m"] >> m;
	fsread["back_m"] >> back_m;
	fsread["pixel_um"] >> pixel_um;

	vector<Point2f> midPoint;
	vector<Point2f> midCannyPoint;
	Mat srcimg = imread("E:\\公司文件\\安测半导体检测\\标定图\\5-19\\平面标定板（静）\\1\\10.bmp", 1);
	Mat sideImg = imread("E:\\公司文件\\安测半导体检测\\标定图\\5-19\\平面标定板（静）\\3\\10.bmp", 1);
	Mat cloneImg = srcimg.clone();
	Mat cloneImg1 = sideImg.clone();
	cvtColor(cloneImg, cloneImg, COLOR_BGR2GRAY);
	cvtColor(cloneImg1, cloneImg1, COLOR_BGR2GRAY);
	threshold(cloneImg, cloneImg, value, 255, 1);
	threshold(cloneImg1, cloneImg1, value, 255, 1);
	getCenterPoint(cloneImg, midPoint);
	getCtByCanny(srcimg, midCannyPoint);
	int threshNum = midPoint.size();
	int CannyNum = midCannyPoint.size();
	//string boardType = "静";
	string boardType = "动";
	string address1 = "E:\\公司文件\\安测半导体检测\\标定图\\5-19\\平面标定板（" + boardType + "）\\1";
	string address2 = "E:\\公司文件\\安测半导体检测\\标定图\\5-19\\平面标定板（" + boardType + "）\\3";
	vector<string> fn;
	vector<string> fn2;
	glob(address1, fn);
	glob(address2, fn2);
	vector<vector<float>> all_corner_distance(threshNum);
	vector<vector<float>> all_canny_corner_distance(CannyNum);
	for (int i = 0; i < fn.size(); i++)
	{
		Mat imgM, imgL, dstImg, warpImgL, threshImgM, threshWarpImgL, tempImg;
		imgM = imread(fn[i], 1);
		imgL = imread(fn2[2], 1);
		if (imgM.empty() || imgL.empty())
		{
			return -1;
		}
		Mat cloneImgL = imgL.clone();
		if (imgM.channels() == 3 && imgL.channels() == 3)
		{
			cvtColor(imgM, imgM, COLOR_BGR2GRAY);
			cvtColor(imgL, imgL, COLOR_BGR2GRAY);
		}
		warpPerspective(imgL, warpImgL, back_m, imgL.size());
		threshold(imgM, threshImgM, value, 255, 1);
		threshold(warpImgL, threshWarpImgL, value, 255, 1);
		vector<Point2f> ptM, ptL, ptM_Thresh, ptL_Thresh;
		
		bool thresh2pt = true;//阈值求中心点
		bool canny2pt = false;//轮廓求中心点
		if (thresh2pt)
		{
			getCenterPoint(threshImgM, ptM_Thresh);
			getCenterPoint(threshWarpImgL, ptL_Thresh);
			if (ptM_Thresh.size() != ptL_Thresh.size())
			{
				return -2;
			}
			getPixelError(ptM_Thresh, ptL_Thresh, all_corner_distance, fs, m, pixel_um);
		}
		if (canny2pt)
		{
			getCtByCanny(imgM, ptM);
			getCtByCanny(warpImgL, ptL);
			if (ptM.size() != ptL.size())
			{
				return -3;
			}
			getPixelError(ptM, ptL, all_canny_corner_distance, fs, m, pixel_um);
		}

		cout << "image " << i << " done.." << endl;
		Log(fs,i);
	}
	float maxthrValue = getValue(srcimg, midPoint, all_corner_distance);
	fs <<"阈值： " << threshNum << " 个点" + boardType + "态重复性精度为 " << maxthrValue << " um" << endl;
	float maxCannyValue = getValue(srcimg, midPoint, all_canny_corner_distance);
	fs <<"Canny： " << CannyNum << " 个点" + boardType + "态重复性精度为 " << maxCannyValue << " um" << endl;
	fs << "---------------------------------------" << endl;
	return 0;
}

void getCenterPoint(Mat& srcImg, vector<Point2f>& centerPt)
{
	int width = srcImg.cols;
	int	height = srcImg.rows;
	vector<vector<Point>> contours;
	vector<Point2f> allCenter;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
	morphologyEx(srcImg, srcImg, MORPH_OPEN, kernel);

	findContours(srcImg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		Moments mm = moments((Mat)contours[i]);
		Point2f crossPt;
		float area = mm.m00;
		crossPt.x = mm.m10 / mm.m00;
		crossPt.y = mm.m01 / mm.m00;
		bool bRange;
		//bRange = crossPt.x > width / 20 * 6 && crossPt.x < width / 20 * 14 && crossPt.y >height / 20 * 6 && crossPt.y < height / 20 * 14 && area >1500 && area < 5000;//49 point
		//bRange = crossPt.x > width / 20 * 5 && crossPt.x < width / 20 * 15 && crossPt.y >height / 20 * 5 && crossPt.y < height / 20 * 15 && area >1500 && area < 5000;//81 point
		//bRange = crossPt.x > width / 20 * 4 && crossPt.x < width / 20 * 16 && crossPt.y >height / 20 * 4 && crossPt.y < height / 20 * 16 && area >1500 && area < 5000;//121 point
		//bRange = crossPt.x > width / 20 * 3 && crossPt.x < width / 20 * 17 && crossPt.y >height / 20 * 3 && crossPt.y < height / 20 * 17 && area >1500 && area < 5000;//169 point
		bRange = crossPt.x > width / 20 * 2 && crossPt.x < width / 20 * 18 && crossPt.y >height / 20 * 2 && crossPt.y < height / 20 * 18 && area >1500 && area < 5000;//225 point
		if (bRange)
		{
			circle(srcImg, crossPt, 20, Scalar(0), -1);
			allCenter.push_back(crossPt);
		}
	}

	//排序
	vector<Point2f> pixCenterClone;
	vector<Point2f> tempPoint;
	vector<vector<Point2f>> sortPixCenter;
	pixCenterClone.assign(allCenter.begin(), allCenter.end());
	do
	{
		for (int j = pixCenterClone.size() - 1; j > -1; j--)
		{
			if (abs(pixCenterClone[0].y - pixCenterClone[j].y) < 10)
			{
				tempPoint.push_back(pixCenterClone[j]);
				pixCenterClone.erase(pixCenterClone.begin() + j);
			}
		}
		sort(tempPoint.begin(), tempPoint.end(), comparePoint);
		sortPixCenter.push_back(tempPoint);
		tempPoint.clear();

	} while (pixCenterClone.size() > 5);
	for (int i = 0; i < sortPixCenter.size(); i++)
	{
		for (int j = 0; j < sortPixCenter[i].size(); j++)
		{
			centerPt.push_back(sortPixCenter[i][j]);
		}
	}
}

void getCtByCanny(Mat& inputImg, vector<Point2f>& centerPt)
{
	Mat srcImg = inputImg.clone();
	Mat grayImg = inputImg.clone();
	int width = srcImg.cols;
	int height = srcImg.rows;
	if (srcImg.channels() == 3)
	{
		cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
	}
	else
	{
		cvtColor(srcImg, srcImg, COLOR_GRAY2BGR);
	}
	Mat cannyImg;
	Canny(grayImg, cannyImg, 30, 90);
	DefectsRepair(cannyImg, 4);
	vector<vector<Point>> contours;
	vector<Point2f> allCrossPt;
	vector<Point2f> allCenter;
	findContours(cannyImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 150)
		{
			Moments mm = moments(contours[i]);
			Point2f crossPt;
			crossPt.x = mm.m10 / mm.m00;
			crossPt.y = mm.m01 / mm.m00;
			bool bRange;
			//bRange = mm.m00 > 2000 && crossPt.x > width / 20 * 6 && crossPt.x < width / 20 * 14 && crossPt.y >height / 20 * 6 && crossPt.y < height / 20 * 14;//49 point
			//bRange = mm.m00 > 2000 && crossPt.x > width / 20 * 5 && crossPt.x < width / 20 * 15 && crossPt.y >height / 20 * 5 && crossPt.y < height / 20 * 15;//81 point
			//bRange = mm.m00 > 2000 && crossPt.x > width / 20 * 4 && crossPt.x < width / 20 * 16 && crossPt.y >height / 20 * 4 && crossPt.y < height / 20 * 16;//121 point
			//bRange = mm.m00 > 2000 && crossPt.x > width / 20 * 3 && crossPt.x < width / 20 * 17 && crossPt.y >height / 20 * 3 && crossPt.y < height / 20 * 17;//169 point
			bRange = mm.m00 > 2000 && crossPt.x > width / 20 * 2 && crossPt.x < width / 20 * 18 && crossPt.y >height / 20 * 2 && crossPt.y < height / 20 * 18;//225 point
			if (bRange)
			{
				circle(srcImg, crossPt, 20, Scalar(0,0,255), -1);
				allCenter.push_back(crossPt);
			}
		}
	}
	//排序
	vector<Point2f> pixCenterClone;
	vector<Point2f> tempPoint;
	vector<vector<Point2f>> sortPixCenter;
	pixCenterClone.assign(allCenter.begin(), allCenter.end());
	do
	{
		for (int j = pixCenterClone.size() - 1; j > -1; j--)
		{
			if (abs(pixCenterClone[0].y - pixCenterClone[j].y) < 10)
			{
				tempPoint.push_back(pixCenterClone[j]);
				pixCenterClone.erase(pixCenterClone.begin() + j);
			}
		}
		sort(tempPoint.begin(), tempPoint.end(), comparePoint);
		sortPixCenter.push_back(tempPoint);
		tempPoint.clear();

	} while (pixCenterClone.size() > 5);
	for (int i = 0; i < sortPixCenter.size(); i++)
	{
		for (int j = 0; j < sortPixCenter[i].size(); j++)
		{
			centerPt.push_back(sortPixCenter[i][j]);
		}
	}
	waitKey(1);
}

void getPixelError(vector<Point2f>& ptM, vector<Point2f>& ptL, vector<vector<float>>& all_corner_distance, fstream& fs, Mat& m, double& pixel_um)
{
	vector<Point2f> tra_ptL, tra_ptM;
	perspectiveTransform(ptM, tra_ptM, m);
	perspectiveTransform(ptL, tra_ptL, m);

	float meanLength = 0;
	float subDisp = 0;
	vector<float> corner_distance;

	for (int j = 0; j < ptM.size(); j++)
	{

		float length = pixel_um * sqrt(pow(tra_ptM[j].x - tra_ptL[j].x, 2) + pow(tra_ptM[j].y - tra_ptL[j].y, 2));
		meanLength += length;
		corner_distance.push_back(length);
	}
	meanLength = meanLength / ptM.size();///拟定平面的深度
	for (int i = 0; i < ptM.size(); i++)
	{
		all_corner_distance[i].push_back(corner_distance[i] - meanLength);
	}
}

float getValue(Mat& inputImg, vector<Point2f> ptM, vector<vector<float>>all_corner_distance)
{
	vector<int> pointSite;
	vector<float> allSubValue;
	for (int i = 0; i < all_corner_distance.size(); i++)
	{
		float maxValue = *max_element(all_corner_distance[i].begin(), all_corner_distance[i].end());
		float minValue = *min_element(all_corner_distance[i].begin(), all_corner_distance[i].end());
		float subValue = maxValue - minValue;
		if (subValue > 0)
		{
			allSubValue.push_back(subValue);
			circle(inputImg, ptM[i], 20, Scalar(0, 0, 255), -1);
		}
	}
	float maxValue = *max_element(allSubValue.begin(), allSubValue.end());
	int maxSite = max_element(allSubValue.begin(), allSubValue.end()) - allSubValue.begin();
	return maxValue;
}

void DefectsRepair(Mat& pBinary, float fDisThre)
{
	int x, y, i, j;
	int x0, x1, x2;
	int n, dx, dy, nSize;
	uchar* pLine[3];
	float fDistance;
	Point ptPoint;
	// 轮廓端点坐标
	std::vector<Point> vecPoint;
	// 执行条件

	for (y = 1; y < pBinary.rows - 1; y++)
	{
		pLine[0] = (pBinary.ptr<uchar>(y - 1));
		pLine[1] = (pBinary.ptr<uchar>(y));
		pLine[2] = (pBinary.ptr<uchar>(y + 1));
		for (x = 1; x < pBinary.cols - 1; x++)
		{
			if (pLine[1][x] > 0)
			{
				n = 0;
				x0 = x - 1;
				x1 = x;
				x2 = x + 1;
				if (pLine[0][x0] != pLine[0][x1]) n++;
				if (pLine[0][x1] != pLine[0][x2]) n++;
				if (pLine[0][x2] != pLine[1][x2]) n++;
				if (pLine[1][x2] != pLine[2][x2]) n++;
				if (pLine[2][x2] != pLine[2][x1]) n++;
				if (pLine[2][x1] != pLine[2][x0]) n++;
				if (pLine[2][x0] != pLine[1][x0]) n++;
				if (pLine[1][x0] != pLine[0][x0]) n++;
				// 孤立点
				if (n == 0)
				{
					if (pLine[1][x] != pLine[0][x0])
					{
						ptPoint.x = x;
						ptPoint.y = y;
						vecPoint.push_back(ptPoint);
					}
				}
				// 轮廓端点
				else if (n == 2)
				{
					ptPoint.x = x;
					ptPoint.y = y;
					vecPoint.push_back(ptPoint);
				}
			}
		}
	}
	// 缺陷修补
	nSize = (int)vecPoint.size();
	for (i = 0; i < nSize - 1; i++)
	{
		for (j = i + 1; j < nSize; j++)
		{
			dx = vecPoint[i].x - vecPoint[j].x;
			dy = vecPoint[i].y - vecPoint[j].y;
			fDistance = (float)(dx * dx + dy * dy);
			if (fDistance <= fDisThre * fDisThre)
			{
				line(pBinary, vecPoint[i], vecPoint[j], Scalar(255));
			}
		}
	}

}

void Log(fstream& out , int i)
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	out << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond;
	out << "       image " << i << " done.." << endl;
}