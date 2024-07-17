#pragma once
// functions on cccb splines
#include "rossy_utils/io/io_utilities.hpp"
class CCCBSplineVec;

class CCCBTrajManager{ 
    public:
        CCCBTrajManager();
        ~CCCBTrajManager(){ delete spline_t2q_;}

        void setSpline();

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
        CCCBSplineVec* spline_t2q_;
};