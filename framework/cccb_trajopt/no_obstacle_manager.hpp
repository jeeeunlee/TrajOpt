#pragma once

#include "framework/obstacle_manager.hpp"


class NoObstacleManager : public ObstacleManager {
    public:
        NoObstacleManager():ObstacleManager(){};
        virtual ~NoObstacleManager(){}

        virtual void updateObstacleCoeff(
            const std::vector<Eigen::VectorXd> &joint_configs,
            Eigen::MatrixXd &U, 
            Eigen::VectorXd &d){ // do nothing
            };
};