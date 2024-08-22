
#include "rossy_utils/robot_system/robot_system.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"
#include "framework/cccb_trajopt/cccb_traj_planner.hpp"
#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "framework/cccb_trajopt/cccb_trajopt_solver.hpp"
#include "framework/cccb_trajopt/no_obstacle_manager.hpp"
#include "framework/cccb_trajopt/rtcl_obstacle_manager.hpp"
#include "cccb_traj_planner.hpp"


// CCCB-spline based Trajectory optimization planner
CCCBTrajOptPlanner::CCCBTrajOptPlanner(RobotSystem* _robot, 
    CCCBTrajManager* _cccb_traj, 
    ObstacleManager* _obstacle_manager, 
    int _link_idx) 
    :Planner(_robot), link_idx_(_link_idx), threshold_pinv_(0.2) {
    rossy_utils::pretty_constructor(1, "CCCBspline Trajectory Optimization Planner");

    cccb_traj_ = _cccb_traj;
    obstacle_manager_ = _obstacle_manager;

    n_dof_ = robot_->getNumDofs();
    trajopt_solver_ = new CCCBTrajOptSolver(cccb_traj_, obstacle_manager_);    

    // set default jerk limit value as 1000
    vel_limit_ = robot_->GetVelocityUpperLimits();
    acc_limit_ = Eigen::VectorXd::Constant(n_dof_, 6.);     
    jerk_limit_ = Eigen::VectorXd::Constant(n_dof_, 1000.); 
}

CCCBTrajOptPlanner::~CCCBTrajOptPlanner() { }

bool CCCBTrajOptPlanner::doPlanning(PLANNING_COMMAND* planning_cmd) {  
    // user_cmd: WPT_DATA()
    std::cout<<" CCCBTrajOptPlanner::doPlanning " <<std::endl;
    if(b_planned_== false){
        b_planned_ = true;
        b_planned_firstvisit_=false;   
        obstacle_manager_->setGrippedBox(planning_cmd->gripped_box);
        obstacle_manager_->setObstacles(planning_cmd->obstacles);
        // this will save the trajectory in cccb_traj_
        bool soln_exist = trajopt_solver_->solve(planning_cmd);        
        
        if(soln_exist){
            start_time_ = current_time_;
            planned_time_ = cccb_traj_->getMotionPeriod();
            end_time_ = start_time_ + planned_time_;
            std::cout<<" start_time_ = "<<start_time_<<std::endl;
            std::cout<<" planned_time_ = "<<planned_time_<<std::endl;
            std::cout<<" end_time_ = "<<end_time_<<std::endl; 
        }     
        return soln_exist;
    }else{
        std::cout<<" Planned trajectory isn't over"<<std::endl;
        return false;
    }
}

bool CCCBTrajOptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        cccb_traj_->getCommand(current_time_ - start_time_ , q_cmd);        
        return true; 
    }
    return false;
}

bool CCCBTrajOptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd,
                                    Eigen::VectorXd& qdot_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        cccb_traj_->getCommand(current_time_ - start_time_ , 
                               q_cmd,  qdot_cmd);   
        return true; 
    }
    return false;        
}

bool CCCBTrajOptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd,
                                    Eigen::VectorXd& qdot_cmd,
                                    Eigen::VectorXd& qddot_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            std::cout<<" b_planned_firstvisit_ = false /  ";
            std::cout<<" current_time_ = " << current_time_ <<std::endl;
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        cccb_traj_->getCommand(current_time_ - start_time_ , 
                                q_cmd, qdot_cmd, qddot_cmd);

        return true; 
    }
    return false;        
}

void CCCBTrajOptPlanner::setVelLimit(const Eigen::VectorXd &vm){
    rossy_utils::pretty_print(vm, std::cout, "setVelLimit");
    vel_limit_ = vm; 
} 
void CCCBTrajOptPlanner::setAccLimit(const Eigen::VectorXd &am){
    rossy_utils::pretty_print(am, std::cout, "setAccLimit");
    acc_limit_ = am; 
}
void CCCBTrajOptPlanner::setJerkLimit(const Eigen::VectorXd &jm){
    rossy_utils::pretty_print(jm, std::cout, "setJerkLimit");
    jerk_limit_ = jm; 
}

void CCCBTrajOptPlanner::setAlpha(double alpha)
{
    trajopt_solver_->alpha_ = alpha;
}

void CCCBTrajOptPlanner::getPlannedResult(SOLUTION * soln)
{
    trajopt_solver_-> getKnotValues(soln);
}
