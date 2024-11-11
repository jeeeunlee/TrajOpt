#pragma once


#include <Eigen/Dense>
#include "rossy_utils/io/io_utilities.hpp"

template <typename Scalar>
class RobotSystem;

class SensorData {
   public:
    SensorData(int n_qv) {
        elapsedtime = 0.;
        q = Eigen::VectorXf::Zero(n_qv);
        qdot = Eigen::VectorXf::Zero(n_qv);
    }
    virtual ~SensorData() {}

    double elapsedtime;
    Eigen::VectorXf q;
    Eigen::VectorXf qdot;
};

class RobotCommand {
   public:
    RobotCommand(int n_qv) {
        q = Eigen::VectorXf::Zero(n_qv);
        qdot = Eigen::VectorXf::Zero(n_qv);
        qddot = Eigen::VectorXf::Zero(n_qv);
        jtrq = Eigen::VectorXf::Zero(n_qv);
    }
    virtual ~RobotCommand() {}

    Eigen::VectorXf q;
    Eigen::VectorXf qdot;
    Eigen::VectorXf qddot;
    Eigen::VectorXf jtrq;
};

class EnvInterface{
  protected:
    RobotSystem<float>* robot_;

  public:
    EnvInterface() {}
    virtual ~EnvInterface(){}

    // Get Command through Test
    virtual void updateState(SensorData* _sensor_data) = 0;
    virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data) = 0;
    virtual bool doPlanning(void* user_cmd) = 0;
    
};
