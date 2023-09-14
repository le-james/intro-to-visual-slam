#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

// residual
struct CurveFittingCost
{
    // member variables
    const double m_x, m_y;
 
    // constructors
    CurveFittingCost(double x, double y) : m_x(x), m_y(y) {}

    // templated overload of operator ()
    template<typename T>
    // abc an array of the estimated parameters - abc is an argument in AddResidualBlock() below
    // update the residual
    bool operator()(const T *const abc, T *residual) const
    {
        residual[0] = T(m_y) - ceres::exp(abc[0] * T(m_x) * T(m_x) + abc[1] * T(m_x) + abc[2]);
        return true;
    }
};

int main()
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;

    int N = 100;

    double w_sigma = 1.0;

    double inv_sigma = 1.0/w_sigma;

    // generate the ground thruth data using rng
    cv::RNG rng;

    std::vector<double> x_data, y_data;
    
    for(int i = 0; i < N; i++)
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma));
    }


    // store the estimated coefficients in an array- this is the parameter block
    double abc[3] = {ae, be, ce};


    // construct the problem in ceres
    ceres::Problem myProblem;
    // setup the cost function - this also takes the derivative to get the jacobian
    for(int i = 0; i < N; i++)
    {
        // setup the cost function using ceres
        // AutoDiffCostFunction is a template class that takes in the functor, num of residuals, num of parameters to solve
        // its constructor takes in the functor
        // the number of residuals and parameters are for the auto diff - it needs to know the dimensions        
        // number of residul equations = 1 
        // number of parameters to solve for = 3
        ceres::CostFunction *myCostFunction = new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 3> (new CurveFittingCost(x_data[i], y_data[i]));  

        // includes the error terms into the cost function - basically taking the summation
        // myCostFunction is the residual block
        // abc is the parameter block
        myProblem.AddResidualBlock(myCostFunction, nullptr, abc);
    }

    // ceres solver options
    // there are lots of parameters that can be adjusted
    ceres::Solver::Options myOptions;
    // use cholesky to solve the normal equations
    myOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    // print the progress of the solver
    myOptions.minimizer_progress_to_stdout = true;

    // object to store the results
    ceres::Solver::Summary mySummary;


    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // solve the least squares problem
    ceres::Solve(myOptions, &myProblem, &mySummary);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


    // time used
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "time used: " << time_used.count() << "seconds" << std::endl;


    // output of the solver
    std::cout << mySummary.BriefReport() << std::endl;
    std::cout << "Estimated a, b, c = ";
    for(auto p:abc)
    {
        std::cout << p << " ";
    }

}


