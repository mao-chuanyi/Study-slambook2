//
// Created by mao on 3/26/23.
//
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(){
    MatrixXd A = MatrixXd::Random(10, 10);
    Matrix3d I = Matrix3d::Identity();
    //取出A矩阵右下角的3阶矩阵
    I = A.block<3,3>(A.cols() - 3, A.rows() - 3);
    cout << "A = \n" << A << endl;
    cout << "I = \n" << I << endl;
 }