//
// Created by mao on 23-4-10.
//
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main (int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0; //噪声sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;
    
    vector<double> x_data, y_data;
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }
    //开始迭代
    int iterations = 100;//迭代次数
    double cost = 0, lastCost = 0;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)
    {
        Matrix3d H = Matrix3d::Zero(); //Hessian
        Vector3d b = Vector3d::Zero(); //bias
        cost = 0;

        for (int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i]; //第i个数据点
            double error = xi - exp(ae * xi * xi + be * xi + ce); //误差
            Vector3d J;//雅可比
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = exp(ae * xi * xi + be * xi + ce);

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }
        //求解Hx=b
        Vector3d dx = H.ldlt().solve(b);//对于正定矩阵，使用cholesky分解来解方程
        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }
        if (iter > 0 && iter >= lastCost)
        {
            cout << "cost:" << cost << ">=lastCost" << ",break" << endl;
            break;
        }
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;
        cout << "total cost: " << cost << ",\t\tupdate: " << dx.transpose() << "\t\testimated params: " << ae << "," << be << "," << ce << endl;

    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << "seconds." << endl;
    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;

    return 0;
}