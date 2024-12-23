
#pragma once

#include "framework/interface.hpp"
#include "framework/user_command.hpp"

class Planner;
class Controller;
class Clock;
class CCCBTrajManager;
class ObstacleManager;

class NoColTestInterface : public EnvInterface {
   protected:
      double running_time_;

      int link_idx_;      
      std::string robot_urdf_path_;      

      RobotCommand* cmd_;
      SensorData* data_;
      PLANNING_COMMAND* plan_cmd_;

      Planner* planner_;
      Controller* controller_;
      CCCBTrajManager* cccb_traj_;
      ObstacleManager* obstacle_manager_;      
      
      Clock* clock_;     

      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jacc_;
      Eigen::VectorXd cmd_jtrq_;

   public:
      NoColTestInterface(const std::string_view urdf_path);
      ~NoColTestInterface();
      
      virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data);
      virtual bool doPlanning(void* user_cmd);
      virtual void updateState(SensorData* _sensor_data);

      void updateVelLimit(const Eigen::VectorXd &vm);
      void updateAccLimit(const Eigen::VectorXd &am);
      void updateJerkLimit(const Eigen::VectorXd &jm);  
      void updateAlpha(double alpha);

      void getPlannedTrajectory(const double& time_step,
                           TRAJ_DATA* traj_data);

      void getPlannedResult(SOLUTION * soln);


   private:
      void updateRobotSystem(SensorData * data); 
      void saveData(SensorData* _sensor_data, RobotCommand* _command_data);
};
