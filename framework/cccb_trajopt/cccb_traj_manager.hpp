#pragma once
// functions on cccb splines
#include "rossy_utils/io/io_utilities.hpp"
class CCCBSplineVec;

class CCCBTrajManager{ 
    public:
        CCCBTrajManager();
        ~CCCBTrajManager();

        void setBSpline(const Eigen::VectorXd &pi, 
            const Eigen::VectorXd &pf,
            const std::vector<Eigen::VectorXd> &cp_in);  
        void setTimeDuration(const double & h_in);

        void getCommand(double t, Eigen::VectorXd& q_cmd);
        void getCommand(double t, 
                        Eigen::VectorXd& q_cmd, 
                        Eigen::VectorXd& qdot_cmd);
        void getCommand(double t, 
                        Eigen::VectorXd& q_cmd, 
                        Eigen::VectorXd& qdot_cmd,
                        Eigen::VectorXd& qddot_cmd);       
        double getMotionPeriod();

    public:
        Eigen::MatrixXd findBSpline(
            const std::vector< Eigen::VectorXd > &joint_path);
        Eigen::MatrixXd computeAp(int N, int dim);
        Eigen::VectorXd computebp(int N, int dim, 
                    const Eigen::VectorXd &pi, const Eigen::VectorXd &pf);
        
        // this maps CPs of first-derivative of B-Spline
        Eigen::MatrixXd computeAv2(int N, int dim);
        Eigen::VectorXd computebv2(int N, int dim,
                    const Eigen::VectorXd &pi, const Eigen::VectorXd &pf);
        // this maps each values at knot points 
        Eigen::MatrixXd computeAv(int N, int dim);
        Eigen::VectorXd computebv(int N, int dim,
                    const Eigen::VectorXd &pi, const Eigen::VectorXd &pf);
        Eigen::MatrixXd computeAa(int N, int dim);
        Eigen::VectorXd computeba(int N, int dim, 
                    const Eigen::VectorXd &pi, const Eigen::VectorXd &pf);
        Eigen::MatrixXd computeAj(int N, int dim);
        Eigen::VectorXd computebj(int N, int dim, 
                    const Eigen::VectorXd &pi, const Eigen::VectorXd &pf);

    public:
        CCCBSplineVec* spline_t2q_;    
        
    private:
        Eigen::MatrixXd stack1dMatDim(int dim, const Eigen::MatrixXd &A1d);

};
