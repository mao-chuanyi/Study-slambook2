//
// Created by mao on 3/29/23.
//
#include <iostream>
#include <Eigen/Core>
#include <cmath>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

//Sophus的基本用法
int main (){
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0,0,1)).toRotationMatrix();
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    cout << "SO3 from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO3 from quaternion:\n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    Vector3d so3 = SO3_R.log();//李代数
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 vee =  " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 update = \n" << SO3_update.matrix() << endl;

    cout << "*************************************************" << endl;
    //对SE(3)操作
    Vector3d t(1, 0, 0);//平移
    Sophus::SE3d SE3_Re(R, t);
    Sophus::SE3d SE3_qe(q, t);
    cout << "SE3 from R,t= \n" << SE3_Re.matrix() << endl;
    cout << "SE3 from q,r= \n" << SE3_qe.matrix() << endl;

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_qe.log();
    cout << "se3 = " << se3.transpose() << endl;

    cout << "se3 hat = \n" <<Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)) << endl;
    //更新
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Re;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}