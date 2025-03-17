#pragma once

#include <deque>

#include "framework/planner.hpp"
#include "framework/user_command.hpp"

class CCCBTrajManager;
class CCCBTrajOptSolver;
class ObstacleManager;

// CCCB-spline Trajectory planner
class CCCBTrajOptPlanner: public Planner{
  protected:
    RobotSystem<float>* robot_planner_;
    int link_idx_;
    int n_dof_;
    // updatable through the interface
    Eigen::VectorXf vel_limit_;
    Eigen::VectorXf acc_limit_;
    Eigen::VectorXf jerk_limit_;

    CCCBTrajManager* cccb_traj_;
    CCCBTrajOptSolver* trajopt_solver_;
    ObstacleManager* obstacle_manager_;

    float threshold_pinv_;
    
  public:
    CCCBTrajOptPlanner(RobotSystem<float>* _robot, 
                      CCCBTrajManager* _cccb_traj, 
                      ObstacleManager* _obstacle_manager, 
                      int _link_idx);
    ~CCCBTrajOptPlanner();

    bool initPlanner(PLANNING_COMMAND* planning_cmd);
    bool doPlanning(PLANNING_COMMAND* planning_cmd);
    bool getPlannedCommand(Eigen::VectorXf& q_cmd);
    bool getPlannedCommand(Eigen::VectorXf& q_cmd,
                          Eigen::VectorXf& qdot_cmd);
    bool getPlannedCommand(Eigen::VectorXf& q_cmd,
                          Eigen::VectorXf& qdot_cmd,
                          Eigen::VectorXf& qddot_cmd);

    // data setting for interface
    void setVelLimit(const Eigen::VectorXf &vm);   
    void setAccLimit(const Eigen::VectorXf &am);
    void setJerkLimit(const Eigen::VectorXf &jm);

    // for check
    void getPlannedResult(SOLUTION * soln);
    void solveFK(const Eigen::VectorXf &q, 
                  const Eigen::VectorXf &qdot,
                  Eigen::VectorXf &x, 
                  Eigen::VectorXf &xdot);
    
};

