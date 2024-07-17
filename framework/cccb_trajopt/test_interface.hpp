
#pragma once

#include "framework/interface.hpp"
#include "framework/user_command.hpp"

class Planner;
class Controller;
class Clock;
class CCCBTrajManager;

namespace ROBOT_TYPE{
  constexpr int RS020N7L=0;
  constexpr int RS020N7R=1;
  constexpr int RS020N7LV4=2;
  constexpr int RS020N7RV4=3;
  constexpr int RA830=4;
  constexpr int RS020N=5;

  const std::array<std::string, 6> names = {
        "rs020n7L", "rs020n7R", "rs020n7L_V4", "rs020n7R_V4", "ra830", "rs020n"
  };
}


class TestInterface : public EnvInterface {
   protected:
      double running_time_;

      int link_idx_;      
      std::string robot_urdf_path_;
      int robot_type_; // :rs020n, rs020n7L, rs020n7R, ra830

      RobotCommand* cmd_;
      SensorData* data_;
      PLANNING_COMMAND* plan_cmd_;

      Planner* planner_;
      Controller* controller_;
      CCCBTrajManager* cccb_traj_;      
      
      Clock* clock_;     

      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jacc_;
      Eigen::VectorXd cmd_jtrq_;

   public:
      TestInterface(int robot_type);
      ~TestInterface();
      
      virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data);
      virtual bool doPlanning(void* user_cmd);
      virtual void updateState(SensorData* _sensor_data);

      void rePlanning();
      void updateVelLimit(const Eigen::VectorXd &vm);
      void updateAccLimit(const Eigen::VectorXd &am);
      void updateJerkLimit(const Eigen::VectorXd &jm);      

      void getPlannedResult(const double& time_step,
                           TRAJ_DATA* traj_data);


   private:
      void updateRobotSystem(SensorData * data); 
      void saveData(SensorData* _sensor_data, RobotCommand* _command_data);
      void setConfiguration(const std::string& cfgfile);    
};
