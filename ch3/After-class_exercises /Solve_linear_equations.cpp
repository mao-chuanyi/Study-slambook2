//
// Created by mao on 3/26/23.
//
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(){
    int n = 4;
    MatrixXd A(n, n);
    A << 1,5,-1,-1,
         1,-2,1,3,
         3,8,-1,1,
         1,-9,3,7;
    VectorXd x(n), b(n);
    b << -1,3,1,7;
    //高斯消元法（化行阶梯）
    //x = A.colPivHouseholderQr().solve(b);
    //cout << "x = " << x.transpose() << endl;
    //LU分解法
    x = A.lu().solve(b);
    cout << "x = " << x.transpose() << endl;

    return 0;
}