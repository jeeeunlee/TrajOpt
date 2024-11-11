#pragma once
// functions on cccb splines
#include "rossy_utils/io/io_utilities.hpp"

template <typename Scalar>
class CCCBSplineVec;

class CCCBTrajManager{ 
    public:
        CCCBTrajManager();
        ~CCCBTrajManager();

        void setBSpline(const Eigen::VectorXf &pi, 
            const Eigen::VectorXf &pf,
            const std::vector<Eigen::VectorXf> &cp_in);  
        void setTimeDuration(const float & h_in);

        void getCommand(float t, Eigen::VectorXf& q_cmd);
        void getCommand(float t, 
                        Eigen::VectorXf& q_cmd, 
                        Eigen::VectorXf& qdot_cmd);
        void getCommand(float t, 
                        Eigen::VectorXf& q_cmd, 
                        Eigen::VectorXf& qdot_cmd,
                        Eigen::VectorXf& qddot_cmd);       
        float getMotionPeriod();

    public:
        Eigen::MatrixXf findBSpline(
            const std::vector< Eigen::VectorXf > &joint_path);
        Eigen::MatrixXf computeAp(int N, int dim);
        Eigen::VectorXf computebp(int N, int dim, 
                    const Eigen::VectorXf &pi, const Eigen::VectorXf &pf);
        
        // this maps CPs of first-derivative of B-Spline
        Eigen::MatrixXf computeAv2(int N, int dim);
        Eigen::VectorXf computebv2(int N, int dim,
                    const Eigen::VectorXf &pi, const Eigen::VectorXf &pf);
        // this maps each values at knot points 
        Eigen::MatrixXf computeAv(int N, int dim);
        Eigen::VectorXf computebv(int N, int dim,
                    const Eigen::VectorXf &pi, const Eigen::VectorXf &pf);
        Eigen::MatrixXf computeAa(int N, int dim);
        Eigen::VectorXf computeba(int N, int dim, 
                    const Eigen::VectorXf &pi, const Eigen::VectorXf &pf);
        Eigen::MatrixXf computeAj(int N, int dim);
        Eigen::VectorXf computebj(int N, int dim, 
                    const Eigen::VectorXf &pi, const Eigen::VectorXf &pf);

    public:
        CCCBSplineVec<float>* spline_t2q_;    
        
    private:
        Eigen::MatrixXf stack1dMatDim(int dim, const Eigen::MatrixXf &A1d);

};
