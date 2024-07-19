#pragma once


#include <iostream>
#include <Eigen/Dense>
#include "rossy_utils/io/io_utilities.hpp"

class TRAJ_DATA{
  public:
    TRAJ_DATA(){  
      tdata.clear();    
      qdata.clear();
      dqdata.clear();
      xdata.clear();
      dxdata.clear();
      period=0.;
      }
    ~TRAJ_DATA(){}
  public:
    std::vector< double > tdata;
    std::vector< Eigen::VectorXd > qdata;
    std::vector< Eigen::VectorXd > dqdata;
    std::vector< Eigen::VectorXd > xdata;
    std::vector< Eigen::VectorXd > dxdata;
    double period;
};

class PLANNING_COMMAND{
public:
    PLANNING_COMMAND(){      
      joint_path.clear();
      cartesian_path.clear();

      // should be set
      max_joint_speed={};
      max_joint_acceleration={};
      max_joint_jerk={};
      }
    ~PLANNING_COMMAND(){}
  public:
    std::vector< Eigen::VectorXd > joint_path;
    std::vector< Eigen::VectorXd > cartesian_path;
    
    Eigen::VectorXd max_joint_acceleration;
    Eigen::VectorXd max_joint_speed;
    Eigen::VectorXd max_joint_jerk;    
};


class SOLUTION{
public:
    SOLUTION(){    
        h=0.;  
        path.clear();
        velocity.clear();
        acceleration.clear();
        jerk.clear();
      }
    ~SOLUTION(){}
  public:
    double h;
    std::vector< Eigen::VectorXd > path;
    std::vector< Eigen::VectorXd > velocity;
    std::vector< Eigen::VectorXd > acceleration;
    std::vector< Eigen::VectorXd > jerk;       
};


class WPT_DATA{
  public:
    WPT_DATA(){
      b_cartesian = true;
      data.clear();}
    ~WPT_DATA(){}

    int getsize(){return data.size();}
    Eigen::VectorXd getdata(unsigned int i){
      if(i>0 && i<data.size()) return data[i];
      else return Eigen::VectorXd::Zero(0);
    }
  public:
    bool b_cartesian;
    std::vector< Eigen::VectorXd > data;
}; // t x d data

class VEC_DATA{
  public:
  Eigen::VectorXd data;
  VEC_DATA(){}
  ~VEC_DATA(){}
}; // d x 1 data