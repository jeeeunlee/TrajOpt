#pragma once

#include <deque>
#include <Eigen/Dense>

template <typename Scalar>
class RobotSystem;

class PLANNING_COMMAND;

class Planner{
  protected:
    RobotSystem<float>* robot_;
    double current_time_ =0.;
    double start_time_=0.;
    double planned_time_=0.;
    double end_time_=0.;

    bool b_planned_=false;
    bool b_planned_firstvisit_=false;
    bool b_planner_initialize_=false;

  public:
    Planner(RobotSystem<float>* _robot){
        robot_ = _robot; }
    virtual ~Planner(){}

    virtual bool initPlanner(PLANNING_COMMAND* _user_cmd) = 0;
    virtual bool doPlanning(PLANNING_COMMAND* _user_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXf& q_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXf& q_cmd,
                                Eigen::VectorXf& qdot_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXf& q_cmd,
                                Eigen::VectorXf& qdot_cmd,
                                Eigen::VectorXf& qddot_cmd) = 0;

    void reset(){b_planned_=false; b_planner_initialize_=false;}
    void updateTime(double _t){
      current_time_ = _t;
      if(b_planned_ && 
          (current_time_ > end_time_)){
              b_planned_ = false;
          }
    }
};