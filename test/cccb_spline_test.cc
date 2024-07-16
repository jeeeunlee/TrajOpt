#include <gtest/gtest.h>

#include "rossy_utils/math/cccb_spline.hpp"
#include "rossy_utils/io/io_utilities.hpp"


TEST(CCCBsplineTest, Check1dCase){
    // check 1d cases
    double pi(-1.), pf(1.), h(0.5);
    std::vector<double> cps{-0.3, 0., 0.7};
    CCCBSpline sp1d(pi,pf,cps, h);   
    double tend = (double)(sp1d.getNumIntervals())*h;

    // print results
    double tstep = h/5.;
    double t(0.);    
    while(t<tend + tstep){
        
        std::cout<< t << ","
            << sp1d.evaluate(t) << ", " 
            << sp1d.evaluateFirstDerivative(t) << ", "
            << sp1d.evaluateSecondDerivative(t) << std::endl;
        t += tstep;
    }

    // check boundary conditions
    EXPECT_DOUBLE_EQ(sp1d.evaluate(0), pi);
    EXPECT_DOUBLE_EQ(sp1d.evaluate(tend), pf);
    EXPECT_DOUBLE_EQ(sp1d.evaluateFirstDerivative(0), 0.);
    EXPECT_DOUBLE_EQ(sp1d.evaluateFirstDerivative(tend), 0.);
    EXPECT_DOUBLE_EQ(sp1d.evaluateSecondDerivative(0), 0.);
    EXPECT_DOUBLE_EQ(sp1d.evaluateSecondDerivative(tend), 0.);

} 
    
TEST(CCCBsplineTest, Check3dCase){
    // check Vector cases
    Eigen::VectorXd vpi{{-1,-1,-1}};
    Eigen::VectorXd vpf{{1,2,3}};    
    std::vector<Eigen::VectorXd> vcps{ 0.9*vpi + 0.1*vpf, 0.5*vpi + 0.4*vpf, -0.1*vpi + 1.1*vpf };
    CCCBSplineVec sp3d(vpi, vpf, vcps);  
    double tend = (double)(sp3d.getNumIntervals())*1.;  
    
    // print results
    double tstep = 1./5.;
    double t(0.);    
    while(t<tend + tstep)
    {
        std::cout<< t << ",";
        for(int d(0); d<sp3d.getDim(); d++){            
            std::cout << sp3d.evaluate(t)(d) << ", " 
                << sp3d.evaluateFirstDerivative(t)(d) << ", "
                << sp3d.evaluateSecondDerivative(t)(d) << ", ";
        }
        t += tstep;
        std::cout<< std::endl;
    }

    // check boundary conditions
    double EPS = 1e-15;
    EXPECT_NEAR((sp3d.evaluate(0)-vpi).norm(), 0., EPS);
    EXPECT_NEAR((sp3d.evaluate(tend)-vpf).norm(), 0., EPS);
    EXPECT_NEAR((sp3d.evaluateFirstDerivative(0)-Eigen::VectorXd::Zero(3)).norm(), 0., EPS);
    EXPECT_NEAR((sp3d.evaluateFirstDerivative(tend)-Eigen::VectorXd::Zero(3)).norm(), 0., EPS);
    EXPECT_NEAR((sp3d.evaluateSecondDerivative(0)-Eigen::VectorXd::Zero(3)).norm(), 0., EPS);
    EXPECT_NEAR((sp3d.evaluateSecondDerivative(tend)-Eigen::VectorXd::Zero(3)).norm(), 0., EPS);
}