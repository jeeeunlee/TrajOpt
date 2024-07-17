#include "framework/cccb_trajopt/simple_controller.hpp"
#include "framework/cccb_trajopt/cccb_traj_planner.hpp"
#include "rossy_utils/robot_system/robot_system.hpp"

// Test Controller
 
SimpleController::SimpleController(RobotSystem* _robot, 
                              Planner* _planner, CCCBTrajManager* _cccb_traj)
  : Controller(_robot, _planner) { 
    rossy_utils::pretty_constructor(1, "Simple Controller");

    cccb_traj_ = _cccb_traj;
    ndof_ = robot_->getNumDofs();
    q_lower_ = robot_->GetPositionLowerLimits();
    q_upper_ = robot_->GetPositionUpperLimits();
    qdot_lower_  = robot_->GetVelocityLowerLimits();
    qdot_upper_ = robot_->GetVelocityUpperLimits();
}

SimpleController::~SimpleController(){}

// Get Command through Test
void SimpleController::getCommand(RobotCommand* cmd){
  static bool b_first = true;
  if(b_first){
    q_cmd_last_ = robot_->getQ();
    b_first = false;
  }   

  Eigen::VectorXd q_cmd, qdot_cmd, qddot_cmd;
  bool bplan = planner_->getPlannedCommand(q_cmd, qdot_cmd, qddot_cmd);
  if(bplan){
    q_cmd_last_ = q_cmd;
    enforcePositionLimits(q_cmd, qdot_cmd);
    enforceVelocityLimits(q_cmd, qdot_cmd);
    cmd->q=q_cmd;
    cmd->qdot=qdot_cmd;
    cmd->qddot=qddot_cmd;
  }else {
    cmd->q=q_cmd_last_;  
    cmd->qdot=Eigen::VectorXd::Zero(q_cmd_last_.size());  
    cmd->qddot=Eigen::VectorXd::Zero(q_cmd_last_.size());  
  }
}

void SimpleController::enforcePositionLimits(Eigen::VectorXd& q_cmd, 
                                          Eigen::VectorXd& qdot_cmd){
    // check limits
    for(int i(0); i<ndof_; ++i){
      q_cmd(i) = q_cmd(i) > q_lower_(i) ? q_cmd(i) : q_lower_(i);
      q_cmd(i) = q_cmd(i) < q_upper_(i) ? q_cmd(i) : q_upper_(i);
    }
}

void SimpleController::enforceVelocityLimits(Eigen::VectorXd& q_cmd, 
                                          Eigen::VectorXd& qdot_cmd){
    // check limits
    for(int i(0); i<ndof_; ++i){
      qdot_cmd(i) = qdot_cmd(i) > qdot_lower_(i) ? qdot_cmd(i) : qdot_lower_(i);
      qdot_cmd(i) = qdot_cmd(i) < qdot_upper_(i) ? qdot_cmd(i) : qdot_upper_(i);
    }
}

