#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

    // rotation matrix 90 degree around z axis
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Quaterniond q(R);


    // SO3 and so3 stuff

    // construct SO3 double
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);             
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;
 

    // logarithmic map to get Lie algebra
    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;

    // vector to skew-symmetric matrix
    cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
    // skew-symmetric matrix to vector
    cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;


    // update by using the perturbation model
    Vector3d update_so3(0.5, 0, 0);
    // Sophus::SO3d::exp() only takes a rotation vector
    // SO3_update is basically delta_R*R (small rotation)
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;


    cout << "*******************************" << endl;


    // SE3 and se3 stuff

    // translation vector (move 1 along x axis)
    Vector3d t(1, 0, 0);           
    // construct SE3
    Sophus::SE3d SE3_Rt(R, t);          
    Sophus::SE3d SE3_qt(q, t);           
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;


    // Lie algebra is 6d vector - have to create a typedef
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    // similarly to SO3 to se3 using logarithmic map
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;

    // vector to skew-symmetric matrix
    cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
    // skew-symmetric matrix to vector
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;


    // update by using the perturbation model
    Vector6d update_se3;
    update_se3.setZero();
    cout << update_se3.transpose() << endl;
    update_se3(0) = 1e-4;
    // or
    // update_se3(0,0) = 1e-4;
    cout << update_se3.transpose() << endl;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}
