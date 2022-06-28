#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;


double Entropy(Mat img ,string name)
{
	//开辟内存
	double temp[256] = { 0.0 };

	// 计算每个像素的累积值
	for (int m = 0; m < img.rows; m++)
	{// 有效访问行列的方式
		const uchar* t = img.ptr<uchar>(m);
		for (int n = 0; n < img.cols; n++)
		{
			int i = t[n];
			temp[i] = temp[i] + 1;
		}
	}

	// 计算每个像素的概率
	for (int i = 0; i < 256; i++)
	{
		temp[i] = temp[i] / (img.rows*img.cols);
	}

	double result = 0;
	// 计算图像信息熵
	for (int i = 0; i < 256; i++)
	{
		if (temp[i] == 0.0)
			result = result;
		else
			result = result - temp[i] * (log(temp[i]) / log(2.0));
	}

	cout <<name<< "的信息熵为：" << result << endl;
	return result;
}

int main(int argc,char* argv[])
{


}