#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main ()
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d(0,0,1) );   

    cout.precision(3);
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;  
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Vector3d v(1,2,3);

    std::cout << "out " << v(0,0) << std::endl;

    // // dont need to convert a angleaxis to a matrix first to rotate a vector
    // Eigen::Vector3d v_rotated = rotation_vector * v;
    // cout << "v(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // v_rotated = rotation_matrix * v;
    // cout<<"v(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 );
    // cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // // euclidean transformation 4x4
    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();    
    // // set rotation matrix         
    // T.rotate ( rotation_vector );     
    // // set translation vector                               
    // T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     
    // cout << "Transform matrix = \n" << T.matrix() <<endl;

    // // v_transformed = R*v + t
    // Eigen::Vector3d v_transformed = T*v;                  
    // cout << "v tranformed = " << v_transformed.transpose() << endl;


    // // quaternions
    // Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    // cout << "quaternion = \n" << q.coeffs().transpose() << endl;   

    // q = Eigen::Quaterniond ( rotation_matrix );
    // cout << "quaternion = \n" << q.coeffs().transpose() << endl;

    // v_rotated = q*v;
    // cout << "v(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    return 0;
}
