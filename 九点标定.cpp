#include<iostream>
#include<opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;


void OnBnClickedBtn9pointcalib();
Mat LocalAffineEstimate(const vector<Point2f>& shape1, const vector<Point2f>& shape2, bool fullAfine);

int main()
{
    OnBnClickedBtn9pointcalib();

    return 0;
}

void OnBnClickedBtn9pointcalib()
{
    std::vector<Point2f> points_camera;
    std::vector<Point2f> points_robot;

    points_camera.push_back(Point2f(1516.14f, 1119.48f));
    points_camera.push_back(Point2f(1516.24f, 967.751f));
    points_camera.push_back(Point2f(1668.55f, 967.29f));
    points_camera.push_back(Point2f(1668.44f, 1118.93f));
    points_camera.push_back(Point2f(1668.54f, 1270.57f));
    points_camera.push_back(Point2f(1516.22f, 1271.37f));
    points_camera.push_back(Point2f(1364.21f, 1272.15f));
    points_camera.push_back(Point2f(1364.08f, 1120.42f));
    points_camera.push_back(Point2f(1364.08f, 968.496f));

    points_robot.push_back(Point2f(35.7173f, 18.8248f));
    points_robot.push_back(Point2f(40.7173f, 18.8248f));
    points_robot.push_back(Point2f(40.7173f, 13.8248f));
    points_robot.push_back(Point2f(35.7173f, 13.8248f));
    points_robot.push_back(Point2f(30.7173f, 13.8248f));
    points_robot.push_back(Point2f(30.7173f, 18.8248f));
    points_robot.push_back(Point2f(30.7173f, 23.8248f));
    points_robot.push_back(Point2f(35.7173f, 23.8248f));
    points_robot.push_back(Point2f(40.7173f, 23.8248f));

    //变换成 2*3 矩阵
    Mat  affine, inliers;
    estimateAffine2D(points_camera, points_robot, inliers).convertTo(affine, CV_32F);
    if (affine.empty())
    {
        affine = LocalAffineEstimate(points_camera, points_robot, true);
    }
    float A, B, C, D, E, F;
    A = affine.at<float>(0, 0);
    B = affine.at<float>(0, 1);
    C = affine.at<float>(0, 2);
    D = affine.at<float>(1, 0);
    E = affine.at<float>(1, 1);
    F = affine.at<float>(1, 2);

    //坐标转换
    //Point2f src = Point2f(1652.6f, 1351.27f);
    //Point2f src = Point2f(100, 100);
    Point2f src = Point2f(1516.14f, 1119.48f);
    Point2f dst;
    dst.x = A * src.x + B * src.y + C;
    dst.y = D * src.x + E * src.y + F;

    //RMS 标定偏差
    std::vector<Point2f> points_Calc;
    double sumX = 0, sumY = 0;
    for (int i = 0; i < points_camera.size(); i++)
    {
        Point2f pt;
        pt.x = A * points_camera[i].x + B * points_camera[i].y + C;
        pt.y = D * points_camera[i].x + E * points_camera[i].y + F;
        points_Calc.push_back(pt);
        sumX += pow(points_robot[i].x - points_Calc[i].x, 2);
        sumY += pow(points_robot[i].y - points_Calc[i].y, 2);
    }
    double rmsX, rmsY;
    rmsX = sqrt(sumX / points_camera.size());
    rmsY = sqrt(sumY / points_camera.size());

}

Mat LocalAffineEstimate(const vector<Point2f>& shape1, const vector<Point2f>& shape2,
    bool fullAfine)
{
    Mat out(2, 3, CV_32F);
    int siz = 2 * (int)shape1.size();

    if (fullAfine)
    {
        Mat matM(siz, 6, CV_32F);
        Mat matP(siz, 1, CV_32F);
        int contPt = 0;
        for (int ii = 0; ii < siz; ii++)
        {
            Mat therow = Mat::zeros(1, 6, CV_32F);
            if (ii % 2 == 0)
            {
                therow.at<float>(0, 0) = shape1[contPt].x;
                therow.at<float>(0, 1) = shape1[contPt].y;
                therow.at<float>(0, 2) = 1;
                therow.row(0).copyTo(matM.row(ii));
                matP.at<float>(ii, 0) = shape2[contPt].x;
            }
            else
            {
                therow.at<float>(0, 3) = shape1[contPt].x;
                therow.at<float>(0, 4) = shape1[contPt].y;
                therow.at<float>(0, 5) = 1;
                therow.row(0).copyTo(matM.row(ii));
                matP.at<float>(ii, 0) = shape2[contPt].y;
                contPt++;
            }
        }
        Mat sol;
        solve(matM, matP, sol, DECOMP_SVD);
        out = sol.reshape(0, 2);
    }
    else
    {
        Mat matM(siz, 4, CV_32F);
        Mat matP(siz, 1, CV_32F);
        int contPt = 0;
        for (int ii = 0; ii < siz; ii++)
        {
            Mat therow = Mat::zeros(1, 4, CV_32F);
            if (ii % 2 == 0)
            {
                therow.at<float>(0, 0) = shape1[contPt].x;
                therow.at<float>(0, 1) = shape1[contPt].y;
                therow.at<float>(0, 2) = 1;
                therow.row(0).copyTo(matM.row(ii));
                matP.at<float>(ii, 0) = shape2[contPt].x;
            }
            else
            {
                therow.at<float>(0, 0) = -shape1[contPt].y;
                therow.at<float>(0, 1) = shape1[contPt].x;
                therow.at<float>(0, 3) = 1;
                therow.row(0).copyTo(matM.row(ii));
                matP.at<float>(ii, 0) = shape2[contPt].y;
                contPt++;
            }
        }
        Mat sol;
        solve(matM, matP, sol, DECOMP_SVD);
        out.at<float>(0, 0) = sol.at<float>(0, 0);
        out.at<float>(0, 1) = sol.at<float>(1, 0);
        out.at<float>(0, 2) = sol.at<float>(2, 0);
        out.at<float>(1, 0) = -sol.at<float>(1, 0);
        out.at<float>(1, 1) = sol.at<float>(0, 0);
        out.at<float>(1, 2) = sol.at<float>(3, 0);
    }
    return out;
}