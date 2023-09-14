#include <iostream>

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

// vertex - 3D vector
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        // read eigens doc on this - cpp 17 and newer compilers don't need this so i am good to exclude it
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // setToOrginImpl and oplusImpl are in the base class of BaseVertex which is Vertex
        // _estimate is defined in BaseVertex

        // virtual void setToOriginImpl() override
            // virtual and override are not really needed
            // override will let you know if you typed the method name wrong
            // virtual doesn't really do anything here
        // or
        virtual void setToOriginImpl() override
        {
            // set the state (estimate) of the vertex to zero
            _estimate << 0, 0, 0; 
        }

        virtual void oplusImpl(const double *update) override
        {
            // update is the increment term deltaX
            _estimate += Eigen::Vector3d(update);
        }

        // read() and write() are part of base class Vertex - they are pure virtual functions
        // bool read(std::istream &in) override {}
        // bool write(std::ostream &out) const override {}

        // in the g2o pdf it doesn't include the {}, it just ends the line with a ;
            // this cause a undefined vtable error during build
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
};

// edge - 1D error term connected to one vertex
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
    public:
        double _x;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // inherit the base class constructor using an initializer list
            // BaseUnaryEdge() will run when a CurveFittingEdge object is created
        CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

        // define the error term computation
        void computeError()
        {
            // _vertices is just a vector that stores pointers of type Vertex
                // cast the vector type from std::vector<Vertex*> to type CurveFittingVertex
                // BaseUnaryEdge -> BaseFixedSizedEdge inherits _vertices from BaseEdge
                // _vertices declared in BaseEdge -> BaseVertex -> OptimizableGraph::Vertex -> HyperGraph::Edge::Vertex
            //_vertices is set below at "add edges" section
            const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);

            const Eigen::Vector3d abc = v->estimate();

            // _error and _measurement are is in BaseUnaryEdge -> BaseFixedSizedEdge -> BaseEdge
                // they are both scalars
                // _measurement gets added below its y_data to fit the function
            // not sure why they wrote it as abc(0,0) instead of just abc(0)
            _error(0,0) = _measurement - std::exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
        }

        // jacobian
        void linearizeOplus() override
        {
            const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);

            const Eigen::Vector3d abc = v->estimate();

            // analytically derived the jacobian
            double y = exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
            _jacobianOplusXi[0] = -_x*_x*y;
            _jacobianOplusXi[1] = -_x*y;
            _jacobianOplusXi[2] = -y;
        }

        // read() and write() are part of base class Vertex
        // bool read(std::istream &in) override {}
        // bool write(std::ostream &out) const override {}

        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}
};


int main()
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0/w_sigma;
    cv::RNG rng;

    std::vector<double> x_data, y_data;
    for(int i = 0; i < N; i++)
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma));
    }

    // BlockSolver sets up the problem (stores the linear system) and LienarSolver solves the problem
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // have to use an older version of g2o for this to work (2017) version
    // these two is basically the auto solver condensed
        // g2o::make_unique<LinearSolverType> *linearsolverdense = g2o::make_unique<LinearSolverType();
        // g2o::make_unique<BlockSolverType> *blocksolvertype = g2o::make_unique<BlockSolverType>(linearsolverdense);
    // choose the optimization method: GN, LM, DogLeg
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    // the below is a solution for the above not working because of "g2o" has no member "make_unique" error
        // it was because the Find<package>.cmake for g2o wasn't add in the cmakelists.txt

    // // linear solver
    // std::unique_ptr< g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> >
    //     linearSolver = std::make_unique< g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> >();
    // // or
    // // std::unique_ptr< g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> >
    // //     linearSolver(new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>());

    // // block solver
    // std::unique_ptr<BlockSolverType> solverPtr = std::make_unique<BlockSolverType>(std::move(linearSolver));
    // // or
    // // std::unique_ptr<BlockSolverType> solverPtr(new BlockSolverType(std::move(linearSolver)));

    // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solverPtr));

    // graph optimizer
    g2o::SparseOptimizer myOptimizer;

    // set the algorithm
    myOptimizer.setAlgorithm(solver);

    // print the results
    myOptimizer.setVerbose(true);


    // add vertex
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(ae,be,ce));
    v->setId(0);
    myOptimizer.addVertex(v);

    // add edges
    for(int i = 0; i < N; i++)
    {
        CurveFittingEdge *e = new CurveFittingEdge(x_data[i]);

        e->setId(i);

        // set the _vertices member variable at index 0 of the vector
        e->setVertex(0,v);

        e->setMeasurement(y_data[i]);

        // the information matrix is the inverse of the covariance of the gaussian distribution
        e->setInformation(Eigen::Matrix<double,1,1>::Identity() * 1/(w_sigma*w_sigma));
        
        myOptimizer.addEdge(e);
    }


    std::cout << "starting optimization" << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    myOptimizer.initializeOptimization();
    myOptimizer.optimize(10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout << "Solve time cost = " << time_used.count() << "seconds." << std::endl;

    // print results
    Eigen::Vector3d abc_estimate = v->estimate();

    std::cout << "Estimate model: " << abc_estimate.transpose() << std::endl;

}