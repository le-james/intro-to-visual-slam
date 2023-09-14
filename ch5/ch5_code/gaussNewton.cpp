#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{
    // the groud truth values
    double ar = 1.0, br = 2.0, cr = 1.0;

    // the initial guesses
    double ae = 2.0, be = -1.0, ce = 5.0;

    // number of data points
    int N = 100;

    // noise/weight
    double w_sigma = 1.0;

    double inv_sigma = 1.0/w_sigma;

    // random noise generator
    cv::RNG rng;
    // generate data points
    std::vector<double> x_data, y_data;
    for(int i = 0; i < N; i++)
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma));
    }


    // gauss newton optimization

    // number of iterations to find the minimum
    int iteration = 100;
    double cost = 0, prevCost = 0;

    // optimization start time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // start optimization
    for(int iter = 0; iter < iteration; iter++)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        cost = 0;

        // compute jacobian, hessian and cost
        for(int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i];

            double error = yi - exp(ae*xi*xi + be*xi + ce);

            // jacobian
            Eigen::Vector3d J;
            // partial of error wrt a
            J[0] = -xi*xi*exp(ae*xi*xi + be*xi + ce);
            // partial of error wrt b
            J[1] = -xi*exp(ae*xi*xi + be*xi + ce);
            // partial of error wrt c
            J[2] = -exp(ae*xi*xi + be*xi + ce);

            H += inv_sigma*inv_sigma*J*J.transpose();
            b += -inv_sigma*inv_sigma*J*error;

            cost += error*error;
        }

        // since H is a symmetric matrix use the cholsky to linear solve for x
        Eigen::Vector3d dx = H.ldlt().solve(b);

        // check if dx contains non a number
        for(int i = 0; i < dx.size(); i++)
        {
             if(isnan(dx[i])) break;
        }

        // if(isnan(dx[0]))
        // {
        //     std::cout << "result is nan!" << std::endl;
        //     break;
        // }

        // not a good check for convergence but just an example
        // if cost > prevCost then you past the minimum
        // if cost = prevCost then the minimum has be achieved
        if(iter > 0 && cost >= prevCost)
        {
            std::cout << "BREAK " << "cost: " << cost << " >= prevCost: " << prevCost << std::endl;
            std::cout << "if cost > prevCost then something is wrong (cost was decreasing then increased)" << std::endl;
            break;
        }

        // update the estimate x_k+1 = x_k + dx
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        prevCost = cost;

        std::cout 
            << "Total Cost: " << cost 
            << ", dx vector: " << dx.transpose() 
            << " Estimate Params: " << ae << ", " << be << ", " << ce
            << " iter: " << iter
            << std::endl;
    }

    // optimization end time
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    // convert from time_point type to duration<double> type
    std::chrono::duration<double> time_used = 
        std::chrono::duration_cast< std::chrono::duration<double> >(t2-t1);

    std::cout << "Solve Time: " << time_used.count() << " seconds" << std::endl;
    std::cout << "Estimated Parameters ae, be, ce: " << ae << ", " << be << ", " << ce << std::endl;
}