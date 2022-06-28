#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>  
using namespace cv;


//读入图像
Mat org = imread("testImg\\百度水印.png");
Mat img, tmp;
Mat src = org;
Mat dst = Mat(src.size(), src.type(), CV_8UC3);
static Point pre_pt;//初始坐标 （选中区域左上角） 
static Point cur_pt;//实时坐标 （选中区域右下角） 

void on_mouse(int event, int x, int y, int flags, void* ustc);
int main()
{

	org.copyTo(img);
	org.copyTo(tmp);
	namedWindow("img", WINDOW_AUTOSIZE);//定义一个img窗口  
	setMouseCallback("img", on_mouse);//调用回调函数  

	waitKey(0);
	return 0;
}
void on_mouse(int event, int x, int y, int flags, void* ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
{

	char temp[16];//定义坐标显示字符数组
	if (event == EVENT_LBUTTONDOWN)//左键按下，读取初始坐标
	{


		//获取选中区域矩形左上角坐标
		pre_pt = Point(x, y);
		imshow("img", img);
	}
	else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))//左键没有按下的情况下,鼠标移动  
	{
		img.copyTo(tmp);//将img复制到临时图像tmp上，用于显示实时坐标  
		sprintf_s(temp, "(%d,%d)", x, y);//坐标
		cur_pt = Point(x, y);//获取实时坐标
		putText(tmp, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));//实时显示鼠标移动的坐标  
		imshow("img", tmp);
	}
	else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划矩形  
	{

		imshow("img", img);
	}
	else if (event == EVENT_LBUTTONUP)//左键松开，将在图像上划矩形  
	{

		//获取实时坐标（选中区域矩形右下角）
		cur_pt = Point(x, y);

		//对选中区域进行像素替换，偏移3个像素，根据实际情况调节
		for (int i = min(pre_pt.y, cur_pt.y); i < max(cur_pt.y, pre_pt.y); i++)
		{
			for (int j = min(pre_pt.x, cur_pt.x); j < max(cur_pt.x, pre_pt.x); j++)
			{
				src.at<Vec3b>(i, j)[0] = src.at<Vec3b>(i, j - 3)[0];
				src.at<Vec3b>(i, j)[1] = src.at<Vec3b>(i, j - 3)[1];
				src.at<Vec3b>(i, j)[2] = src.at<Vec3b>(i, j - 3)[2];

			}
		}

		//对选中区域周围进行平滑处理
		Mat imageroi = src(Range(pre_pt.y - 3, cur_pt.y + 3), Range(pre_pt.x - 3, cur_pt.x + 3));
		Mat tempimg = src(Rect(Point(pre_pt.x - 3 ,pre_pt.y - 3),Point(cur_pt.x + 3 ,cur_pt.y + 3)));
		imshow("temp", tempimg);
		GaussianBlur(imageroi, imageroi, Size(19, 19), 0, 0);
		//medianBlur(imageroi, imageroi, 3);

		bilateralFilter(src, dst, 15, 30, 9);

		//标记选中区域
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), FILLED);
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
		//显示结果
		namedWindow("dst", WINDOW_AUTOSIZE);
		imshow("dst", dst);


	}

}


//double Entropy(Mat& img, string name)
//{
//	int width = img.cols;
//	int height = img.rows;
//	//开辟内存
//	double temp[256] = { 0.0 };
//
//	// 计算每个像素的累积值
//	for (int m = 0; m < height; m++)
//	{// 有效访问行列的方式
//		const uchar* p = img.ptr<uchar>(m);
//		for (int n = 0; n < width; n++)
//		{
//			int i = p[n];
//			temp[i] = temp[i] + 1;
//		}
//	}
//
//	// 计算每个像素的概率
//	for (int i = 0; i < 256; i++)
//	{
//		temp[i] = temp[i] / (width * height);
//	}
//
//	double result = 0;
//	// 计算图像信息熵
//	for (int i = 0; i < 256; i++)
//	{
//		if (temp[i] == 0.0)
//			result = result;
//		else
//			result = result - temp[i] * (log(temp[i]) / log(2.0));
//	}
//
//	cout << name << "的信息熵为：" << result << endl;
//	return result;
//}
