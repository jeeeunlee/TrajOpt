#pragma once

#include "framework/controller.hpp"

class Planner;

// Test Controller
class SimpleController: public Controller{  
  public:   
    SimpleController(RobotSystem<float>* _robot, Planner* _planner);
    ~SimpleController();

    // Get Command through Test
    void getCommand(RobotCommand* _cmd);

    Eigen::VectorXf q_cmd_last_;

  private:
    void enforcePositionLimits(Eigen::VectorXf& q_cmd, 
                              Eigen::VectorXf& qdot_cmd);
    void enforceVelocityLimits(Eigen::VectorXf& q_cmd, 
                              Eigen::VectorXf& qdot_cmd);
    int ndof_;
    Eigen::VectorXf q_lower_;
    Eigen::VectorXf q_upper_;
    Eigen::VectorXf qdot_lower_;
    Eigen::VectorXf qdot_upper_;


};