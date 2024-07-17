// functions on cccb splines

#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "rossy_utils/math/cccb_spline.hpp"
#include "cccb_traj_manager.hpp"

CCCBTrajManager::CCCBTrajManager(){
    rossy_utils::pretty_constructor(1, "TrajectoryManager");
    // spline_t2q_.initialize()
    spline_t2q_ = new CCCBSplineVec();
}

void CCCBTrajManager::setSpline(){
    // set spline_t2q_
}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd){
    q_cmd = spline_t2q_->evaluate(t);
}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd, 
    Eigen::VectorXd & qdot_cmd) {
    q_cmd = spline_t2q_->evaluate(t);
    qdot_cmd = spline_t2q_->evaluateFirstDerivative(t);    
}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd, 
    Eigen::VectorXd & qdot_cmd, 
    Eigen::VectorXd & qddot_cmd) {
    q_cmd = spline_t2q_->evaluate(t);
    qdot_cmd = spline_t2q_->evaluateFirstDerivative(t); 
    qddot_cmd = spline_t2q_->evaluateSecondDerivative(t);
}


double CCCBTrajManager::getMotionPeriod()
{
    return spline_t2q_->getMotionPeriod();
}