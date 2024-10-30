#pragma once

#include "framework/obstacle_manager.hpp"
#include "framework/user_command.hpp"

namespace rtcl{
    class RtclInterface;

}

class RtclObstacleManager : public ObstacleManager {
    public:
        RtclObstacleManager(const std::string_view robot_name, 
                        const std::string_view assets_directory);
        ~RtclObstacleManager(){};

        // virtual void setObstacles(const std::vector<OBSTACLE>& obstacles);
        virtual void updateObstacleCoeff(const std::vector<Eigen::VectorXd> &joint_configs,
                                Eigen::MatrixXd &U, Eigen::VectorXd &d);

        void updateSingleJointCoeff(uint dim,
                                    Eigen::MatrixXf& Ut, 
                                    Eigen::VectorXf& dt);
        
        void updateRandomSingleJointCoeff(uint num_constraints,
                                        uint dim,
                                        Eigen::MatrixXf& Ut, 
                                        Eigen::VectorXf& dt);

    protected:        
        rtcl::RtclInterface* rtcl_interface_;
};