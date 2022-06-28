#include"generateDll.h"

void remapping(vector<double> im_in_Remap, vector<double>& im_out_Remap, vector<double>& refocus_image, unsigned short width,
	unsigned short height, unsigned short window_side, unsigned short stereo_diff, double alpha);//�ؾ۽�
void gradient2D(Mat input, Mat& output);//�ݶȼ���

/*********�߽��ж�*********/
int index_x(int x, int width)
{
	if (0 <= x && x < width)
		return x;
	else if (x < 0)
		return 0;
	else
		return width - 1;
}
int index_y(int y, int height)
{
	if (0 <= y && y < height)
		return y;
	else if (y < 0)
		return 0;
	else
		return height - 1;
}



int main()
{
	Mat srcimg = imread("testImg\\lorikeet.jpg", 0);
	const int windowSize = 11;
	if (srcimg.empty())
	{
		return -1;
	}
	Mat dstimg;
	copyMakeBorder(srcimg, dstimg, 10, 10, 10, 10, 1);
	fstream fsX("datax.txt", fstream::in);
	fstream fsY("datay.txt", fstream::in);
	vector<vector<Point>> allLensPt;
	
	if (fsX.is_open() && fsY.is_open())
	{
		for (int i = 0; i < 434; i++)
		{
			vector<Point> lensCenterPt;
			for (int j = 0; j < 541; j++)
			{
				Point pt;
				string a,b;
				getline(fsX, a, '	');
				getline(fsY, b, '	');
				pt.x = 10 + atoi(a.c_str());
				pt.y = 10 + atoi(b.c_str());
				lensCenterPt.push_back(pt);
			}
			waitKey(1);
			allLensPt.push_back(lensCenterPt);
		}
	}
	const int width = allLensPt[0].size();
	const int height = allLensPt.size();
	Mat remap_LFimg = Mat::zeros(Size(width * windowSize, height * windowSize), CV_8U);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			Mat smallLens = dstimg(Rect(allLensPt[i][j].x - windowSize / 2, allLensPt[i][j].y - windowSize / 2, windowSize, windowSize));
			smallLens.copyTo(remap_LFimg(Rect(j * windowSize, i * windowSize, windowSize, windowSize)));
		}
	}

	Mat sub_img = Mat::zeros(Size(width, height), CV_8U);

	//��remap��Ĺⳡͼ������ؾ۽�
	vector<double> in_Remap_array;
	for (size_t j = 0; j < remap_LFimg.cols; j++)
	{
		for (size_t i = 0; i < remap_LFimg.rows; i++)
		{
			in_Remap_array.push_back(remap_LFimg.at<uchar>(i, j));
		}
	}
	vector<double>out_Remap_array;
	out_Remap_array = in_Remap_array;
	vector<double>refocus_img_Array;
	for (size_t i = 0; i < sub_img.rows; i++)
	{
		for (size_t j = 0; j < sub_img.cols; j++)
		{
			refocus_img_Array.push_back(sub_img.at<uchar>(i, j));
		}
	}
	double first_alpha = 0.2;
	double step_alpha = (2.0 - 0.2) / 50;//�ؾ۽���Χ
	vector<Mat>all_grad_img;
	vector<Mat>all_refocus_img;
	vector<Mat> all_var_img;
	int int_alpha_num = 0;


	
	for (first_alpha; first_alpha < 2; first_alpha += step_alpha)
	{
		remapping
		(
			in_Remap_array,//����ԭʼ�Ĺⳡͼ��
			out_Remap_array,//���ӳ��õĹⳡͼ��
			refocus_img_Array,//�ؾ۽�ͼ��
			541,//�ռ�ֱ��ʵĿ�
			434,//�ռ�ֱ��ʵĸ�
			windowSize,//�Ƕȷֱ��ʱ߳�����˵��΢͸��ֱ��
			windowSize / 2,//��С����΢͸���뾶
			first_alpha//ÿ�θı�ƽ���alphaֵ��L'=alpha*L
		);
		Mat refocus_img = Mat::zeros(sub_img.size(), CV_8U);
		for (size_t i = 0; i < 541; i++)
		{
			for (size_t j = 0; j < 434; j++)
			{
				refocus_img.at<uchar>(j, i) = refocus_img_Array[i * 434 + j];
			}
		}
		all_refocus_img.push_back(refocus_img);//�洢�����ؾ۽�ͼ�񣨽����ջ��
		Mat mid_var_img = Mat::zeros(Size(width, height), CV_32F);
		all_var_img.push_back(mid_var_img);
		Mat grad_img;
		gradient2D(refocus_img, grad_img);//����ݶ�ͼ
		blur(grad_img, grad_img, Size(3, 3));//���ݶ�ͼ����ƽ������
		all_grad_img.push_back(grad_img);//�洢�����ݶ�ͼ

		float a = (float)first_alpha;
		string F_alpha = to_string(a);
		putText(refocus_img, F_alpha, Point(1, 55), 0, 0.4, Scalar(255));
		string alpha_num = to_string(int_alpha_num);
		putText(refocus_img, alpha_num, Point(1, 10), 0, 0.4, Scalar(255));
		namedWindow("1", 0);
		imshow("1", refocus_img);
		waitKey(1);
		//if (waitKey(30) == 27)
		//{
		//	break;
		//}

		//if (first_alpha > 0.9)
		//{
		//	waitKey(100);
		//}

		int_alpha_num += 1;
	}

	//�ݶ�ͼ�Ƚϣ���ø���������ֵ
	Mat final_grad_img = Mat::zeros(sub_img.size(), CV_8U);
	Mat final_grad_refocus_img = Mat::zeros(sub_img.size(), CV_8U);
	for (size_t i = 0; i < sub_img.rows; i++)//�������ݶ�ͼ���бȽϣ���ò�ͬ�������ݶ���������ֵ
	{
		for (size_t j = 0; j < sub_img.cols; j++)
		{
			float max_grad = 0;
			for (size_t m = 0; m < all_grad_img.size(); m++)
			{
				float mid = all_grad_img[m].at<float>(i, j);
				if (mid > max_grad)
				{
					max_grad = mid;
					final_grad_img.at<uchar>(i, j) = m;
					final_grad_refocus_img.at<uchar>(i, j) = all_refocus_img[m].at<uchar>(i, j);
				}
			}
		}
	}



	return 0;
}

/*******************�ؾ۽�����ý����ջ***********************/
void remapping(vector<double> im_in_Remap, vector<double>& im_out_Remap, vector<double>& refocus_image, unsigned short width,
	unsigned short height, unsigned short window_side, unsigned short stereo_diff, double alpha)
{
	int                 x, y;//�ռ�ֱ��ʵĳ��Ϳ�
	unsigned int        x_1, x_2, y_1, y_2;//�ı佹ƽ��֮������ֵ���ڵ��ĸ�Ԫ�ص����ϽǺ����½ǵ����ص�����
	int                 i, j;
	double              x_ind, y_ind;//�ı佹ƽ��֮�󣬹��������µĽ�ƽ���ϵ�����ֵ��һ����С��
	double              x_floor, y_floor;
	double              x_1_w, x_2_w, y_1_w, y_2_w;//�µ�����ֵ�������ĸ�����ֵ��Ȩ�أ����ݾ������
	unsigned int        x_1_index, x_2_index, y_1_index, y_2_index;//�����ĸ������x��y����ֵ
	unsigned int        x_index_remap, y_index_remap;//�����Tao������Ĺ�������ԭʼͼ���ؾ۽��ģ����Ժ���
	double              interp_color_R, interp_color_G, interp_color_B;//����ͨ��
	double              output_color_R, output_color_G, output_color_B;//�ؾ۽���ֵ֮�������ͼ��
	unsigned int        height_of_remap, width_of_remap, pixels_of_remap;//ӳ����֮��ԭʼͼ��ߣ������ظ���
	int                 window_size;//�趨��΢͸�����ڴ�С

	window_size = window_side * window_side;//΢͸�����渲�ǵ�����ֵ����

	height_of_remap = height * window_side;//�ӿ׾�ͼ��� * �趨��΢͸��ֱ��
	width_of_remap = width * window_side;//�ӿ׾�ͼ��� * �趨��΢͸��ֱ��
	pixels_of_remap = height_of_remap * width_of_remap;//ӳ����֮���ԭʼͼ�����ظ���

	for (x = 0; x < width; ++x)//�����Ҵ�С���½��б���
		for (y = 0; y < height; ++y)
		{
			output_color_R = 0;//����ͨ��
			output_color_G = 0;
			output_color_B = 0;

			for (i = -stereo_diff; i < stereo_diff + 1; ++i)//�൱��u��v���Ƕȷֱ���
				for (j = -stereo_diff; j < stereo_diff + 1; ++j)
				{
					/******************��һ�������ɵ�***************************/
					x_ind = (-i * (1 - 1 / alpha) + x);//���ݹ�ʽ���µĽ�ƽ����x������
					y_ind = (-j * (1 - 1 / alpha) + y);//ͬ��y����

					x_floor = floor(x_ind);//����ȡ��
					y_floor = floor(y_ind);

					x_1 = index_x(x_floor, width);//���б߽��ж�
					y_1 = index_y(y_floor, height);
					x_2 = index_x(x_floor + 1, width);
					y_2 = index_y(y_floor + 1, height);

					x_1_w = 1 - (x_ind - x_floor);//�����ĸ��������ز�ֵ��Ȩ�أ�Խ��Ȩ��Խ��
					x_2_w = 1 - x_1_w;
					y_1_w = 1 - (y_ind - y_floor);
					y_2_w = 1 - y_1_w;

					x_1_index = i + stereo_diff + (x_1)*window_side;//ԭʼͼ���ؾ۽�
					y_1_index = j + stereo_diff + (y_1)*window_side;
					x_2_index = i + stereo_diff + (x_2)*window_side;
					y_2_index = j + stereo_diff + (y_2)*window_side;

					interp_color_R = y_1_w * x_1_w * im_in_Remap[y_1_index + x_1_index * height_of_remap] +//������ͨ�����в�ֵ
						y_2_w * x_1_w * im_in_Remap[y_2_index + x_1_index * height_of_remap] +
						y_1_w * x_2_w * im_in_Remap[y_1_index + x_2_index * height_of_remap] +
						y_2_w * x_2_w * im_in_Remap[y_2_index + x_2_index * height_of_remap];
					//interp_color_G = y_1_w * x_1_w * im_in_Remap_array[y_1_index + x_1_index * height_of_remap + 1 * pixels_of_remap] +
					//	y_2_w * x_1_w * im_in_Remap_array[y_2_index + x_1_index * height_of_remap + 1 * pixels_of_remap] +
					//	y_1_w * x_2_w * im_in_Remap_array[y_1_index + x_2_index * height_of_remap + 1 * pixels_of_remap] +
					//	y_2_w * x_2_w * im_in_Remap_array[y_2_index + x_2_index * height_of_remap + 1 * pixels_of_remap];
					//interp_color_B = y_1_w * x_1_w * im_in_Remap_array[y_1_index + x_1_index * height_of_remap + 2 * pixels_of_remap] +
					//	y_2_w * x_1_w * im_in_Remap_array[y_2_index + x_1_index * height_of_remap + 2 * pixels_of_remap] +
					//	y_1_w * x_2_w * im_in_Remap_array[y_1_index + x_2_index * height_of_remap + 2 * pixels_of_remap] +
					//	y_2_w * x_2_w * im_in_Remap_array[y_2_index + x_2_index * height_of_remap + 2 * pixels_of_remap];



					// CORRESPONDENCE ANALYSIS
					x_index_remap = i + stereo_diff + (x)*window_side;
					y_index_remap = j + stereo_diff + (y)*window_side;

					im_out_Remap[y_index_remap + x_index_remap * height_of_remap] = floor(interp_color_R);
					//im_out_Remap_array[y_index_remap + x_index_remap * height_of_remap + 1 * pixels_of_remap] = interp_color_G;
					//im_out_Remap_array[y_index_remap + x_index_remap * height_of_remap + 2 * pixels_of_remap] = interp_color_B;

					// DEFOCUS ANALYSIS
					output_color_R = interp_color_R + output_color_R;//��alphaֵ�¸���ͨ������֮��ĵ�ͨ��ͼ��
					//output_color_G = interp_color_G + output_color_G;
					//output_color_B = interp_color_B + output_color_B;

				}
			refocus_image[y + x * height] = floor(output_color_R / window_size);//��ͨ���ؾ۽�ͼ��
			//output_image[y + x * height + 1 * height * width] = output_color_G / window_size;
			//output_image[y + x * height + 2 * height * width] = output_color_B / window_size;

		}
}

/*******************�Խ����ջ�����ݶȼ���**********************/
void gradient2D(Mat input, Mat& output)
{
	Mat Ix(input.size(), CV_32F);
	Mat Iy(input.size(), CV_32F);
	//get Iy
	for (int nrow = 0; nrow < input.rows; nrow++)
	{
		for (int ncol = 0; ncol < input.cols; ncol++)
		{
			if (ncol == 0)
			{
				Ix.at<float>(nrow, ncol) = abs(input.at<uchar>(nrow, 1) - input.at<uchar>(nrow, 0));
			}
			else if (ncol == input.cols - 1)
			{
				Ix.at<float>(nrow, ncol) = abs(input.at<uchar>(nrow, ncol) - input.at<uchar>(nrow, ncol - 1));
			}
			else
			{
				Ix.at<float>(nrow, ncol) = abs((input.at<uchar>(nrow, ncol + 1) - input.at<uchar>(nrow, ncol - 1)) / 2.0);
			}
		}
	}
	//get Ix
	for (int nrow = 0; nrow < input.rows; nrow++)
	{
		for (int ncol = 0; ncol < input.cols; ncol++)
		{
			if (nrow == 0)
			{
				Iy.at<float>(nrow, ncol) = abs(input.at<uchar>(1, ncol) - input.at<uchar>(0, ncol));
			}
			else if (nrow == input.rows - 1)
			{
				Iy.at<float>(nrow, ncol) = abs(input.at<uchar>(nrow, ncol) - input.at<uchar>(nrow - 1, ncol));
			}
			else
			{
				Iy.at<float>(nrow, ncol) = abs((input.at<uchar>(nrow + 1, ncol) - input.at<uchar>(nrow - 1, ncol)) / 2.0);
			}
		}
	}
	magnitude(Ix, Iy, output);
}
