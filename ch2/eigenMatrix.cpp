#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{
    // Eigen::Matrix<double, 50, 50> matrix_NN;
    // matrix_NN = Eigen::MatrixXd::Random(50, 50);

    // Eigen::Matrix<double, 50, 1> v_Nd;
    // v_Nd = Eigen::MatrixXd::Random(50, 1);

    // clock_t time_stt = clock();

    // Eigen::Matrix<double, 50, 1> x = matrix_NN.inverse() * v_Nd;
    // std::cout <<"time use in normal inverse is " << 1000 * (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< std::endl;

    // time_stt = clock();
    // x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    // std::cout <<"time use in Qr decomposition is " << 1000 * (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << std::endl;

    Eigen::Matrix<float,2,3> m = Eigen::Matrix<float,2,3>::Random(2,3);
    Eigen::Matrix<float,3,1> v = Eigen::Matrix<float,3,1>::Random(3,1);
    Eigen::Matrix<float,2,1> out = m * v;
    std::cout << out << std::endl;

}
