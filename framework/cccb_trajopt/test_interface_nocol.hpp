
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
      float running_time_;

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

      Eigen::VectorXf cmd_jpos_;
      Eigen::VectorXf cmd_jvel_;
      Eigen::VectorXf cmd_jacc_;
      Eigen::VectorXf cmd_jtrq_;

   public:
      NoColTestInterface(const std::string_view urdf_path);
      ~NoColTestInterface();
      
      virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data);
      virtual bool doPlanning(void* user_cmd);
      virtual void updateState(SensorData* _sensor_data);

      void updateVelLimit(const Eigen::VectorXf &vm);
      void updateAccLimit(const Eigen::VectorXf &am);
      void updateJerkLimit(const Eigen::VectorXf &jm);  

      void getPlannedTrajectory(const float& time_step,
                           TRAJ_DATA* traj_data);

      void getPlannedResult(SOLUTION * soln);


   private:
      void updateRobotSystem(SensorData * data); 
      void saveData(SensorData* _sensor_data, RobotCommand* _command_data);
};
