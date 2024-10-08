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
    int link_idx_;
    int n_dof_;
    // updatable through the interface
    Eigen::VectorXd vel_limit_;
    Eigen::VectorXd acc_limit_;
    Eigen::VectorXd jerk_limit_;

    CCCBTrajManager* cccb_traj_;
    CCCBTrajOptSolver* trajopt_solver_;
    ObstacleManager* obstacle_manager_;

    double threshold_pinv_;
    
  public:
    CCCBTrajOptPlanner(RobotSystem* _robot, 
                      CCCBTrajManager* _cccb_traj, 
                      ObstacleManager* _obstacle_manager, 
                      int _link_idx);
    ~CCCBTrajOptPlanner();

    bool doPlanning(PLANNING_COMMAND* planning_cmd);
    bool getPlannedCommand(Eigen::VectorXd& q_cmd);
    bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                          Eigen::VectorXd& qdot_cmd);
    bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                          Eigen::VectorXd& qdot_cmd,
                          Eigen::VectorXd& qddot_cmd);

    // data setting for interface
    void setVelLimit(const Eigen::VectorXd &vm);   
    void setAccLimit(const Eigen::VectorXd &am);
    void setJerkLimit(const Eigen::VectorXd &jm);
    void setAlpha(double alpha);

    // for check
    void getPlannedResult(SOLUTION * soln);
    
};

