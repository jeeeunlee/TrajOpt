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
    std::vector< Eigen::VectorXf > qdata;
    std::vector< Eigen::VectorXf > dqdata;
    std::vector< Eigen::VectorXf > xdata;
    std::vector< Eigen::VectorXf > dxdata;
    double period;
};

class OBSTACLE{
  public:
    OBSTACLE(){
      // only box obstacles for now
      // pos(3) + quat(4)
      pose = Eigen::VectorXf::Zero(7);
      pose << 0., 0., 0., 1., 0., 0., 0.; 
      dimension = Eigen::VectorXf::Zero(3);
      type = 0;
      name = "";
    }
    void printInfo() const{
      std::cout << " pose = " << this->pose.transpose() << std::endl;
      std::cout << " dimension = " << this->dimension.transpose() << std::endl;
    }
  public:
    Eigen::VectorXf pose;
    Eigen::VectorXf dimension;
    int type; // 0: box
    std::string name;
};

class GRIPPED_BOX{
    public:
    GRIPPED_BOX(){
      // pos(3) + quat(4)
      pose_from_ee = Eigen::VectorXf::Zero(7);
      pose_from_ee << 0., 0., 0., 1., 0., 0., 0.; 
      dimension = Eigen::VectorXf::Zero(3);
    }
    void printInfo() const{
      std::cout << " gripped box information :" << std::endl;
      std::cout << "\t- pose_from_ee = " << this->pose_from_ee.transpose() << std::endl;
      std::cout << "\t- dimension = " << this->dimension.transpose() << std::endl;
    }
  public:
    Eigen::VectorXf pose_from_ee;
    Eigen::VectorXf dimension;
};

class PLANNING_COMMAND{
public:
    PLANNING_COMMAND(){      
      joint_path.clear();
      cartesian_path.clear();
      obstacles.clear();

      // should be set
      max_joint_speed={};
      max_joint_acceleration={};
      max_joint_jerk={};
      gripped_box=GRIPPED_BOX();
      }
    ~PLANNING_COMMAND(){}
  public:
    std::vector< Eigen::VectorXf > joint_path;
    std::vector< Eigen::VectorXf > cartesian_path;
    
    Eigen::VectorXf max_joint_acceleration;
    Eigen::VectorXf max_joint_speed;
    Eigen::VectorXf max_joint_jerk;
    std::vector< OBSTACLE > obstacles;
    GRIPPED_BOX gripped_box;
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
    std::vector< Eigen::VectorXf > path;
    std::vector< Eigen::VectorXf > velocity;
    std::vector< Eigen::VectorXf > acceleration;
    std::vector< Eigen::VectorXf > jerk;       
};


class WPT_DATA{
  public:
    WPT_DATA(){
      b_cartesian = true;
      data.clear();}
    ~WPT_DATA(){}

    int getsize(){return data.size();}
    Eigen::VectorXf getdata(unsigned int i){
      if(i>0 && i<data.size()) return data[i];
      else return Eigen::VectorXf::Zero(0);
    }
  public:
    bool b_cartesian;
    std::vector< Eigen::VectorXf > data;
}; // t x d data

class VEC_DATA{
  public:
  Eigen::VectorXf data;
  VEC_DATA(){}
  ~VEC_DATA(){}
}; // d x 1 data