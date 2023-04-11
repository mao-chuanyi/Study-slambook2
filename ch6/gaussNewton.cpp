// Created by mao on 23-4-10.
// This program uses Gauss-Newton method to fit a curve
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main (int argc, char **argv)
{
    // true parameters
    double a_true = 1.0, b_true = 2.0, c_true = 1.0;
    // estimated parameters
    double a_est = 2.0, b_est = -1.0, c_est = 5.0;
    // number of data points
    int N = 100;
    // noise sigma
    double w_sigma = 1.0;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng; // random number generator

    vector<double> x_data, y_data; // data vectors
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        // generate noisy y data
        y_data.push_back(exp(a_true * x * x + b_true * x + c_true) + rng.gaussian(w_sigma * w_sigma));
    }
    // start iteration
    int iterations = 100;// iteration times
    double cost = 0, lastCost = 0;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)
    {
        Matrix3d H = Matrix3d::Zero(); // Hessian matrix
        Vector3d b = Vector3d::Zero(); // bias vector
        cost = 0;

        for (int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i]; // the i-th data point
            double error = yi - exp(a_est * xi * xi + b_est * xi + c_est); // error
            Vector3d J;// Jacobian matrix
            J[0] = -xi * xi * exp(a_est * xi * xi + b_est * xi + c_est); // derivative of error w.r.t a_est
            J[1] = -xi * exp(a_est * xi * xi + b_est * xi + c_est); // derivative of error w.r.t b_est
            J[2] = -exp(a_est * xi * xi + b_est * xi + c_est); // derivative of error w.r.t c_est

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;

            cost += error * error;
        }
        // solve Hx=b
        Vector3d dx = H.ldlt().solve(b);// use cholesky decomposition to solve the equation for positive definite matrix
        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }
        if (iter > 0 && cost >= lastCost)
        {
            cout << "cost:" << cost << ">=lastCost" << ",break" << endl;
            break;
        }
        a_est += dx[0];
        b_est += dx[1];
        c_est += dx[2];

        lastCost = cost;
        cout << "total cost: " << cost <<
             ",\t\testimated params: " << a_est << "," << b_est << "," << c_est << endl;
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used =
            chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost: " << time_used.count() << " seconds." << endl;

    cout << "estimated abc: " << a_est << ", " << b_est << ", " << c_est << endl;
    return 0;
}