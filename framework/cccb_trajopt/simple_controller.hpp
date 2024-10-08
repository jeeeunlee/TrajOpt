#pragma once

#include "framework/controller.hpp"

class Planner;

// Test Controller
class SimpleController: public Controller{  
  public:   
    SimpleController(RobotSystem* _robot, Planner* _planner);
    ~SimpleController();

    // Get Command through Test
    void getCommand(RobotCommand* _cmd);

    Eigen::VectorXd q_cmd_last_;

  private:
    void enforcePositionLimits(Eigen::VectorXd& q_cmd, 
                              Eigen::VectorXd& qdot_cmd);
    void enforceVelocityLimits(Eigen::VectorXd& q_cmd, 
                              Eigen::VectorXd& qdot_cmd);
    int ndof_;
    Eigen::VectorXd q_lower_;
    Eigen::VectorXd q_upper_;
    Eigen::VectorXd qdot_lower_;
    Eigen::VectorXd qdot_upper_;


};