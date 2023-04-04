#include <opencv4/opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../../imageBasics/distorted.png";

int main(int argc, char **argv)
{
    // 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
    //相机内参矩阵
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0, cx,
                                                                 0, fy,cy,
                                                                 0, 0, 1);
    //畸变系数
    cv::Mat disCoeffs = (cv::Mat_<double>(4,1) << k1, k2, p1, p2,0.0);
    //输入图像
    cv::Mat image = cv::imread(image_file, 0);
    //畸变调整后的图像
    cv::Mat image_undistorted;
    //计算畸变后的图像,直接使用openCV提供的函数
    cv::undistort(image,image_undistorted,cameraMatrix, disCoeffs);

    cv::imshow("distorted",image);
    cv::imshow("undistorted",image_undistorted);
    cv::waitKey();
    return 0;
}
