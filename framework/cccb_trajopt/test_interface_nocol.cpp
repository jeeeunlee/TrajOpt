

#include <math.h>
#include <stdio.h>
#include <fmt/core.h>
#include <string_view>

#include "rossy_utils/robot_system/robot_system.hpp"
#include "rossy_utils/general/clock.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"

#include "rossy_utils/robot_system/robot_system.hpp"

#include "framework/cccb_trajopt/test_interface_nocol.hpp"
#include "framework/cccb_trajopt/simple_controller.hpp"
#include "framework/cccb_trajopt/cccb_traj_planner.hpp"
#include "framework/cccb_trajopt/cccb_traj_manager.hpp"

#include "framework/cccb_trajopt/no_obstacle_manager.hpp"

NoColTestInterface::NoColTestInterface(const std::string_view urdf_path)
    :EnvInterface(), robot_urdf_path_{urdf_path} {    
    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
    rossy_utils::pretty_constructor(0, "Test Interface");

    // robot_urdf, link_idx_
    // setConfiguration("/etc/opt/dex/truck_rwc_dev/dhc/test.yaml");
    // setConfiguration("config/test.yaml");
    // robot_urdf_path_ =  "/home/jelee/my_ws/TrajOpt/config/urdf_files/franka_panda.urdf";
    // robot_urdf_path_ = urdf_path;
    link_idx_ = 20; // panda_hand    
    
    // class constructors
    robot_ = new RobotSystem(robot_urdf_path_);    
    cccb_traj_ = new CCCBTrajManager();
    obstacle_manager_ = new NoObstacleManager();
    planner_ = new CCCBTrajOptPlanner(robot_, cccb_traj_, obstacle_manager_, link_idx_);
    controller_ = new SimpleController( robot_, planner_);
    clock_ = new Clock();    

    running_time_ = 0.;

    cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
}

NoColTestInterface::~NoColTestInterface() {
    delete robot_;
    delete planner_;
    delete controller_;
    delete clock_;
}

void NoColTestInterface::getCommand(SensorData* _sensor_data, RobotCommand* _command_data) {
    updateRobotSystem(_sensor_data);   
    controller_->getCommand(_command_data);
}

void NoColTestInterface::updateState(SensorData* _sensor_data){
    updateRobotSystem(_sensor_data);
}

void NoColTestInterface::updateRobotSystem(SensorData* data){
    robot_->updateSystem(data->q, data->qdot);
    running_time_ = data->elapsedtime;
    ((CCCBTrajOptPlanner*)planner_)->updateTime(running_time_);
}


bool NoColTestInterface::doPlanning(void* user_cmd){
    std::cout<<"doPlanning start"<<std::endl;    
    clock_->start();
    plan_cmd_ = (PLANNING_COMMAND*)user_cmd;
    bool planned = planner_->doPlanning(plan_cmd_);
    clock_->printElapsedMiliSec("doPlanning=");
    return planned;
}

void NoColTestInterface::updateVelLimit(const Eigen::VectorXd &vm){
    ((CCCBTrajOptPlanner*)planner_)->setVelLimit(vm);
}

void NoColTestInterface::updateAccLimit(const Eigen::VectorXd &am){
    ((CCCBTrajOptPlanner*)planner_)->setAccLimit(am);
}

void NoColTestInterface::updateJerkLimit(const Eigen::VectorXd &jm){
    ((CCCBTrajOptPlanner*)planner_)->setJerkLimit(jm);
}


void NoColTestInterface::updateAlpha(double alpha)
{
    ((CCCBTrajOptPlanner*)planner_)->setAlpha(alpha);
}

void NoColTestInterface::getPlannedTrajectory(const double& time_step,
                                        TRAJ_DATA* traj_data){ 
    std::cout << "getPlannedTrajectory " << std::endl;;
    // generate desired q, qdot
    Eigen::VectorXd q, qdot;
    Eigen::VectorXd x, xdot;
    double t(0.), tend=cccb_traj_->getMotionPeriod();

    // initialize containers
    traj_data->tdata.clear();
    traj_data->qdata.clear();
    traj_data->dqdata.clear();
    traj_data->xdata.clear();
    traj_data->dxdata.clear();
    traj_data->period = tend;    

    t = 0.;
    while(t < tend + time_step){
        cccb_traj_->getCommand(t, q, qdot);
        t += time_step;
        traj_data->tdata.push_back(t);
        traj_data->qdata.push_back(q);
        traj_data->dqdata.push_back(qdot);

        // ((CCCBTrajOptPlanner*)planner_)->solveFK(q, qdot, x, xdot);
        // traj_data->xdata.push_back(x.head(3));
        // traj_data->dxdata.push_back(xdot.head(3));
    }

    // Assume, planned result will be reset
    planner_->reset(); // bplanned = true
}

void NoColTestInterface::getPlannedResult(SOLUTION * soln){
    ((CCCBTrajOptPlanner*)planner_)->getPlannedResult(soln);
}

// SensorData : q, qdot
// RobotCommand : q, qdot, qddot, jtrq
void NoColTestInterface::saveData(SensorData* _sensor_data, RobotCommand* _command_data){    

    rossy_utils::saveValue(running_time_, "test_t");

    // current joint position & joint velocity & acc
    // (1) from robot system
    // Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    // cmd_jpos_ = robot_->getQ();
    // cmd_jvel_ = robot_->getQdot();
    // cmd_jacc_ = (cmd_jvel_- cmd_jvel_prev)/0.001;

    // (2) from SensorData
    // Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    // cmd_jpos_ = _sensor_data->q;
    // cmd_jvel_ = _sensor_data->qdot;
    // cmd_jacc_ = (cmd_jvel_- cmd_jvel_prev)/0.001;

    // (3) from RobotCommand
    Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    cmd_jpos_ = _command_data->q;
    cmd_jvel_ = _command_data->qdot;
    cmd_jacc_ = _command_data->qddot;
   
    // current joint trq
    Eigen::MatrixXd M = robot_->getMassMatrix();
    Eigen::VectorXd cori = robot_->getCoriolisGravity();
    cmd_jtrq_ = M*cmd_jacc_ + cori;    

    rossy_utils::saveVector(cmd_jpos_, "test_q");
    rossy_utils::saveVector(cmd_jvel_, "test_qdot");
    rossy_utils::saveVector(cmd_jacc_, "test_qddot");
    rossy_utils::saveVector(cmd_jtrq_, "test_trq");    

    // current EE position
    Eigen::VectorXd pose;
    rossy_utils::convertIsoToVec7d(
        robot_->getBodyNodeIsometry(link_idx_), pose);
    rossy_utils::saveVector(pose, "test_EE");
    
}
