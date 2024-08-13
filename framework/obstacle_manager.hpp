#pragma once

#include "rossy_utils/io/io_utilities.hpp"
#include "framework/user_command.hpp"

#include <vector>

class ObstacleManager{
    public:
        ObstacleManager(){};
        virtual ~ObstacleManager(){}

        void setObstacles(const std::vector<OBSTACLE>& obstacles){
            obstacles_.clear();
            for (auto &obs: obstacles){
                // check
                // obs.printInfo();
                obstacles_.push_back(obs);        
            }
        };
        virtual void updateObstacleCoeff(
            const std::vector<Eigen::VectorXd> &joint_configs,
            Eigen::MatrixXd &U, 
            Eigen::VectorXd &d) = 0;

    protected:
        std::vector<OBSTACLE> obstacles_;

};