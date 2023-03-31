//
// Created by mao on 3/21/23.
//

#include <iostream>
using namespace std;

#include <ctime>
//Eigen核心部分
#include <Eigen/Core>
//稠密矩阵的代数运算（逆，特征值）
#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

int main(){
    //声明一个2*3的float矩阵
    Matrix<float, 2, 3> matrix_23;
    //这是两种声明三维向量的方式
    Vector3d v_3d;//double
    Matrix<float, 3, 1> vd_3d;

    //3*3的矩阵
    Matrix3d matrix_33 = Matrix3d::Zero();//初始化为0,Zero()是一个静态成员函数，可以通过类名直接调用
    //不确定矩阵大小，通过动态大小的矩阵
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    //更简单的动态矩阵表示声明方式
    MatrixXd matrix_x;

    //下面是对Eigen阵的操作
    //输入数据
    matrix_23 << 1,2,3,4,5,6;
    //输出
    cout << "matrix 2*3 from 1 to 6 : \n" << matrix_23 << endl;
    //用（）访问第二行第二个元素
    cout << "matrix(1,1):" << matrix_23(1,1) << endl;

    //矩阵和向量相乘(实际上是矩阵和矩阵相乘）
    v_3d << 3,2,1;
    vd_3d << 4,5,6;
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;//.cast<double>()是Eigen中的方法，转换矩阵元素类型
    cout << "[1,2,3; 4,5,6] * [3,2,1] :" << result.transpose() << endl;//.transpose()转置输出

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3; 4,5,6] * [4,5,6] :" << result2.transpose() << endl;

    //随机矩阵
    matrix_33 = Matrix3d::Random();
    //各种输出
    cout << "random matrix:\n" << matrix_33 << endl;
    cout << "transpose:\n" << matrix_33.transpose() << endl;//转置
    cout << "sum: " << matrix_33.sum() << endl;//求和
    cout << "trace: " << matrix_33.trace() << endl;//迹
    cout << "times 10: \n" << 10 * matrix_33 << endl;//数乘
    cout << "inverse: \n" << matrix_33.inverse() << endl;//求逆
    cout << "det: " << matrix_33.determinant() << endl;//行列式


    //特征值
    //实对称矩阵必可相似对角化
    //SelfAdjointEigenSolver类的模板，用于解决3x3对称矩阵的特征值和特征向量
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);//AA^T得到一个对称矩阵
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl; //特征值
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl; //特征向量

    //解方程
    //求解matrix_NN * x = v_Nd 方程
    //N的大小由宏定义，它由随机数生成
    //直接求逆是最直接的，但是计算量大
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();//保证matrix_NN是半正定矩阵x^T A x >= 0
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();//计时
    //直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << " ms" << endl;
    cout << "x = " << x.transpose() << endl;

    //通常用矩阵分解来求解，例如QR分解，速度更快
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of Qr decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << " ms" << endl;
    cout << "x = " << x.transpose() << endl;

    //对于正定矩阵，还可以使用cholesky分解来解方程
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << " ms" << endl;
    cout << "x = " << x.transpose()  << endl;

    return 0;
}
