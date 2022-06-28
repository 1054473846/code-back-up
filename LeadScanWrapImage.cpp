#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
	for (int j = 3; j < 4; j++)
	{
		string a = to_string(j);
		string address = "E:\\公司文件\\安测半导体检测\\标定图\\5-19\\平面标定板（动）\\" + a;
		vector<string> fn;
		glob(address, fn);
		for (int i = 0; i < fn.size(); i++)
		{
			Mat srcimg = imread(fn[i]);
			if (srcimg.empty())
			{
				continue;
			}
			int n = fn[i].find_last_of("\\") + 1;
			string picNum = fn[i].substr(n, fn[i].size() - n);
			int width = srcimg.cols;
			int height = srcimg.rows;
			Point2f srcPoint[4];
			Point2f traPoint[4];
			srcPoint[0] = Point(0, 0);
			srcPoint[1] = Point(width, 0);
			srcPoint[2] = Point(width, height);
			//srcPoint[3] = Point(0, height);

			traPoint[0] = Point(0, height);
			traPoint[1] = Point(width, height);
			traPoint[2] = Point(width, 0);
			//traPoint[3] = Point(0, 0);

			//Mat mm = getPerspectiveTransform(srcPoint, traPoint);
			//warpPerspective(srcimg, srcimg, mm, srcimg.size());

			Mat m = getAffineTransform(srcPoint, traPoint);
			Mat dstimg;
			warpAffine(srcimg, dstimg, m, srcimg.size());
			imwrite(fn[i], dstimg);
			waitKey(1);
		}
	}
	

	return 0;
}