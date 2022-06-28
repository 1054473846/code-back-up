#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>  
using namespace cv;


//����ͼ��
Mat org = imread("testImg\\�ٶ�ˮӡ.png");
Mat img, tmp;
Mat src = org;
Mat dst = Mat(src.size(), src.type(), CV_8UC3);
static Point pre_pt;//��ʼ���� ��ѡ���������Ͻǣ� 
static Point cur_pt;//ʵʱ���� ��ѡ���������½ǣ� 

void on_mouse(int event, int x, int y, int flags, void* ustc);
int main()
{

	org.copyTo(img);
	org.copyTo(tmp);
	namedWindow("img", WINDOW_AUTOSIZE);//����һ��img����  
	setMouseCallback("img", on_mouse);//���ûص�����  

	waitKey(0);
	return 0;
}
void on_mouse(int event, int x, int y, int flags, void* ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���  
{

	char temp[16];//����������ʾ�ַ�����
	if (event == EVENT_LBUTTONDOWN)//������£���ȡ��ʼ����
	{


		//��ȡѡ������������Ͻ�����
		pre_pt = Point(x, y);
		imshow("img", img);
	}
	else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_LBUTTON))//���û�а��µ������,����ƶ�  
	{
		img.copyTo(tmp);//��img���Ƶ���ʱͼ��tmp�ϣ�������ʾʵʱ����  
		sprintf_s(temp, "(%d,%d)", x, y);//����
		cur_pt = Point(x, y);//��ȡʵʱ����
		putText(tmp, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));//ʵʱ��ʾ����ƶ�������  
		imshow("img", tmp);
	}
	else if (event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON))//�������ʱ������ƶ�������ͼ���ϻ�����  
	{

		imshow("img", img);
	}
	else if (event == EVENT_LBUTTONUP)//����ɿ�������ͼ���ϻ�����  
	{

		//��ȡʵʱ���꣨ѡ������������½ǣ�
		cur_pt = Point(x, y);

		//��ѡ��������������滻��ƫ��3�����أ�����ʵ���������
		for (int i = min(pre_pt.y, cur_pt.y); i < max(cur_pt.y, pre_pt.y); i++)
		{
			for (int j = min(pre_pt.x, cur_pt.x); j < max(cur_pt.x, pre_pt.x); j++)
			{
				src.at<Vec3b>(i, j)[0] = src.at<Vec3b>(i, j - 3)[0];
				src.at<Vec3b>(i, j)[1] = src.at<Vec3b>(i, j - 3)[1];
				src.at<Vec3b>(i, j)[2] = src.at<Vec3b>(i, j - 3)[2];

			}
		}

		//��ѡ��������Χ����ƽ������
		Mat imageroi = src(Range(pre_pt.y - 3, cur_pt.y + 3), Range(pre_pt.x - 3, cur_pt.x + 3));
		Mat tempimg = src(Rect(Point(pre_pt.x - 3 ,pre_pt.y - 3),Point(cur_pt.x + 3 ,cur_pt.y + 3)));
		imshow("temp", tempimg);
		GaussianBlur(imageroi, imageroi, Size(19, 19), 0, 0);
		//medianBlur(imageroi, imageroi, 3);

		bilateralFilter(src, dst, 15, 30, 9);

		//���ѡ������
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), FILLED);
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
		//��ʾ���
		namedWindow("dst", WINDOW_AUTOSIZE);
		imshow("dst", dst);


	}

}


//double Entropy(Mat& img, string name)
//{
//	int width = img.cols;
//	int height = img.rows;
//	//�����ڴ�
//	double temp[256] = { 0.0 };
//
//	// ����ÿ�����ص��ۻ�ֵ
//	for (int m = 0; m < height; m++)
//	{// ��Ч�������еķ�ʽ
//		const uchar* p = img.ptr<uchar>(m);
//		for (int n = 0; n < width; n++)
//		{
//			int i = p[n];
//			temp[i] = temp[i] + 1;
//		}
//	}
//
//	// ����ÿ�����صĸ���
//	for (int i = 0; i < 256; i++)
//	{
//		temp[i] = temp[i] / (width * height);
//	}
//
//	double result = 0;
//	// ����ͼ����Ϣ��
//	for (int i = 0; i < 256; i++)
//	{
//		if (temp[i] == 0.0)
//			result = result;
//		else
//			result = result - temp[i] * (log(temp[i]) / log(2.0));
//	}
//
//	cout << name << "����Ϣ��Ϊ��" << result << endl;
//	return result;
//}
