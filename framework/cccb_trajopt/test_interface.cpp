

#include <math.h>
#include <stdio.h>
#include <fmt/core.h>
#include <string_view>

#include "rossy_utils/robot_system/robot_system.hpp"
#include "rossy_utils/general/clock.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"

#include "rossy_utils/robot_system/robot_system.hpp"

#include "framework/cccb_trajopt/test_interface.hpp"
#include "framework/cccb_trajopt/simple_controller.hpp"
#include "framework/cccb_trajopt/cccb_traj_planner.hpp"
#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "framework/cccb_trajopt/rtcl_obstacle_manager.hpp"

TestInterface::TestInterface(const std::string_view robot_name, 
const std::string_view assets_directory)
    :EnvInterface() {    
    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
    rossy_utils::pretty_constructor(0, "Test Interface");

    // robot_urdf, link_idx_
    // setConfiguration("/etc/opt/dex/truck_rwc_dev/dhc/test.yaml");
    // setConfiguration("config/test.yaml");
    // robot_urdf_path_ =  "/home/jelee/my_ws/TrajOpt/config/urdf_files/franka_panda.urdf";
    robot_urdf_path_ = fmt::format("{}/urdfs/{}.urdf", assets_directory, robot_name);
    link_idx_ = 20; // panda_hand    
    
    // class constructors
    robot_ = new RobotSystem(robot_urdf_path_);    
    cccb_traj_ = new CCCBTrajManager();
    obstacle_manager_ = new RtclObstacleManager(robot_name, assets_directory);
    planner_ = new CCCBTrajOptPlanner(robot_, cccb_traj_, obstacle_manager_, link_idx_);
    controller_ = new SimpleController( robot_, planner_);
    clock_ = new Clock();    

    running_time_ = 0.;

    cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
}

TestInterface::~TestInterface() {
    delete robot_;
    delete planner_;
    delete controller_;
    delete clock_;
}

void TestInterface::getCommand(SensorData* _sensor_data, RobotCommand* _command_data) {
    updateRobotSystem(_sensor_data);   
    controller_->getCommand(_command_data);
}

void TestInterface::updateState(SensorData* _sensor_data){
    updateRobotSystem(_sensor_data);
}

void TestInterface::updateRobotSystem(SensorData* data){
    robot_->updateSystem(data->q, data->qdot);
    running_time_ = data->elapsedtime;
    ((CCCBTrajOptPlanner*)planner_)->updateTime(running_time_);
}


bool TestInterface::doPlanning(void* user_cmd){
    std::cout<<"doPlanning start"<<std::endl;    
    clock_->start();
    plan_cmd_ = (PLANNING_COMMAND*)user_cmd;
    bool planned = planner_->doPlanning(plan_cmd_);
    clock_->printElapsedMiliSec("doPlanning = ");
    return planned;
}

void TestInterface::updateVelLimit(const Eigen::VectorXd &vm){
    ((CCCBTrajOptPlanner*)planner_)->setVelLimit(vm);
}

void TestInterface::updateAccLimit(const Eigen::VectorXd &am){
    ((CCCBTrajOptPlanner*)planner_)->setAccLimit(am);
}

void TestInterface::updateJerkLimit(const Eigen::VectorXd &jm){
    ((CCCBTrajOptPlanner*)planner_)->setJerkLimit(jm);
}


void TestInterface::updateAlpha(double alpha)
{
    ((CCCBTrajOptPlanner*)planner_)->setAlpha(alpha);
}

void TestInterface::getPlannedTrajectory(const double& time_step,
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

void TestInterface::getPlannedResult(SOLUTION * soln){
    ((CCCBTrajOptPlanner*)planner_)->getPlannedResult(soln);
}

// SensorData : q, qdot
// RobotCommand : q, qdot, qddot, jtrq
void TestInterface::saveData(SensorData* _sensor_data, RobotCommand* _command_data){    

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
