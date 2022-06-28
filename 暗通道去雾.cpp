#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstring>

#define CV_SORT_EVERY_ROW    0
#define CV_SORT_EVERY_COLUMN 1
#define CV_SORT_ASCENDING    0
#define CV_SORT_DESCENDING   16

//CV_SORT_EVERY_ROW + CV_SORT_ASCENDING：对矩阵的每行按照升序排序；
//CV_SORT_EVERY_ROW + CV_SORT_DESCENDING：对矩阵的每行按照降序排序；
//CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING：对矩阵的每列按照升序排序；
//CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING：对矩阵的每列按照降序排序；

using namespace std;
using namespace cv;

void Cout(int& n)
{
	int flag = 1;
	for (int i = 0; i < 7; i++)
	{
		if (n & flag)
		{
			cout << "哒";
		}
		else
		{
			cout << "么";
		}
		flag = flag << 1;
	}
	cout << endl;

}


int main()
{
	for (int i = 0; i < 128; i++)
	{
		Cout(i);
	}
	return 0;


	//Mat image = imread("芯片规格.jpg");
	//if (image.empty())
	//{
	//	cout << "error!";
	//	return 0;
	//}
	//cout << "complete" << endl;
	//imshow("src", image);
	////imshow("src", src);
	////一、求暗通道
	////1.首先对图像进行归一化
	//Mat fImage;
	//image.convertTo(fImage, CV_32FC3, 1.0 / 255.0);//就是在浮点数的图像数据类型中,其灰度值的范围都是在0-1之间的，而原图像是BGR的，所有像素点的灰度值都在0-255之间,所以如果要保持原图像不变，要乘于一个比例因子:(float)(1/255)

	////2.设定运算窗口
	//int hPatch = 15;
	//int vPatch = 15;

	////3.给归一化的图片设定边界
	//Mat fImageBorder;
	////此边界是为了便于我们处理边缘数据,在后续的卷积操作中，由于部分卷积算子的尺寸特殊性,有些边缘像素没办法处理到，所以就要用此api进行边缘的复制
	//copyMakeBorder(fImage, fImageBorder, vPatch / 2, vPatch / 2, hPatch / 2, hPatch / 2, BORDER_REPLICATE);

	////4.分离通道
	//std::vector<Mat> fImageBorderVector(3);
	//split(fImageBorder, fImageBorderVector);//把三个通道push到vector中去

	////5.创建并且计算暗通道
	//Mat darkChannel(image.rows, image.cols, CV_32FC1);//单通道图像
	//double minTemp, minPixel;
	////计算暗通道
	//for (unsigned int r = 0; r < darkChannel.rows; r++)
	//{
	//	for (unsigned int c = 0; c < darkChannel.cols; c++)
	//	{
	//		minPixel = 1.0;
	//		for (std::vector<Mat>::iterator it = fImageBorderVector.begin(); it != fImageBorderVector.end(); it++)
	//		{
	//			Mat roi(*it, Rect(c, r, hPatch, vPatch));
	//			minMaxLoc(roi, &minTemp);//相当于一个打擂算法，在这个窗口里面找最小值
	//			minPixel = std::min(minPixel, minTemp);
	//		}
	//		//打擂结束,设定最小值,把暗通道算出来,并且赋值
	//		darkChannel.at<float>(r, c) = float(minPixel);
	//	}
	//}

	///*这一段代码是用来测试我们的暗通道是否提取成功
	//imshow("dst", darkChannel);
	//Mat darkChannel8U;
	//darkChannel.convertTo(darkChannel8U, CV_8UC1, 255, 0);
	//imwrite("E:/outputdata/darkchannel_h1.jpg", darkChannel8U);
	//success!
	//*/

	////二、通过暗通道来实现A的过程，求的是大气的那个值
	////1.计算出darkChannel中, 前top个亮的值, 论文中取值为0.1 %
	//float top = 0.001;
	//float numberTop = top * darkChannel.rows * darkChannel.cols;
	//Mat darkChannelVectorOneRow = darkChannel.reshape(1, 1);//单通道,一行
	//Mat_<int> darkChannelVectorIndex;
	////Mat_类
	//sortIdx(darkChannelVectorOneRow, darkChannelVectorIndex, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);//获取数组的索引
	////这个函数是用来先对原来的进行一次sort,然后返回这个sort的数组的中每一个元素所对应元素数组的index,再形成一个arrary赋值到第二个参数中去。
	////2.制作掩码，设定一个二值mask
	//Mat mask(darkChannelVectorIndex.rows, darkChannelVectorIndex.cols, CV_8UC1);//注意mask的类型必须是CV_8UC1
	////然后要做的是,找出原图像中亮度在前百分之1的点,把对应的点设置为1,其他的都设置成0
	////然后由于每行是已经通过sortIdx序列化之后的结果
	//for (unsigned int r = 0; r < darkChannelVectorIndex.rows; r++)
	//{
	//	for (unsigned int c = 0; c < darkChannelVectorIndex.cols; c++)
	//	{
	//		if (darkChannelVectorIndex.at<int>(r, c) <= numberTop)
	//			mask.at<uchar>(r, c) = 1;
	//		else
	//			mask.at<uchar>(r, c) = 0;
	//	}
	//}
	//Mat darkChannelIndex = mask.reshape(1, darkChannel.rows);//单通道
	//vector<double> A(3);//分别存取A_b,A_g,A_r
	//vector<double>::iterator itA = A.begin();
	//vector<Mat>::iterator it = fImageBorderVector.begin();
	////2.2在求第三步的t(x)时，会用到以下的矩阵，这里可以提前求出
	//vector<Mat> fImageBorderVectorA(3);
	//vector<Mat>::iterator itAA = fImageBorderVectorA.begin();
	//for (it = fImageBorderVector.begin(); it != fImageBorderVector.end() && itA != A.end() && itAA != fImageBorderVectorA.end(); it++, itA++, itAA++)
	//{
	//	Mat roi(*it, Rect(hPatch / 2, vPatch / 2, darkChannel.cols, darkChannel.rows));
	//	minMaxLoc(roi, 0, &(*itA), 0, 0, darkChannelIndex);//
	//	(*itAA) = (*it) / (*itA); //[注意：这个地方有除号，但是没有判断是否等于0]
	//}

	///*第三步：求t(x)*/
	//Mat darkChannelA(darkChannel.rows, darkChannel.cols, CV_32FC1);
	//float omega = 0.95;//0<w<=1,论文中取值为0.95
	////代码和求darkChannel的时候,代码差不多
	//for (unsigned int r = 0; r < darkChannel.rows; r++)
	//{
	//	for (unsigned int c = 0; c < darkChannel.cols; c++)
	//	{
	//		minPixel = 1.0;
	//		for (itAA = fImageBorderVectorA.begin(); itAA != fImageBorderVectorA.end(); itAA++)
	//		{
	//			Mat roi(*itAA, Rect(c, r, hPatch, vPatch));
	//			minMaxLoc(roi, &minTemp);
	//			minPixel = min(minPixel, minTemp);
	//		}
	//		darkChannelA.at<float>(r, c) = float(minPixel);
	//	}
	//}
	//Mat tx = 1.0 - omega * darkChannelA;
	///*第四步：我们可以求J(x)*/
	//float t0 = 0.1;//论文中取t0 = 0.1
	//Mat jx(image.rows, image.cols, CV_32FC3);
	//for (size_t r = 0; r < jx.rows; r++)
	//{
	//	for (size_t c = 0; c < jx.cols; c++)
	//	{
	//		jx.at<Vec3f>(r, c) = Vec3f((fImage.at<Vec3f>(r, c)[0] - A[0]) / max(tx.at<float>(r, c), t0) + A[0], (fImage.at<Vec3f>(r, c)[1] - A[1]) / max(tx.at<float>(r, c), t0) + A[1], (fImage.at<Vec3f>(r, c)[2] - A[2]) / max(tx.at<float>(r, c), t0) + A[2]);
	//	}
	//}
	//namedWindow("jx", 1);
	//imshow("jx", jx);
	//Mat jx8u;
	//jx.convertTo(jx8u, CV_8UC3, 255, 0);
	//imwrite("芯片规格clear.jpg", jx8u);
	//waitKey(0);
}
