#pragma once

#include "framework/obstacle_manager.hpp"


class NoObstacleManager : public ObstacleManager {
    public:
        NoObstacleManager():ObstacleManager(){};
        virtual ~NoObstacleManager(){}

        virtual void computeCollisionConstraints(
                const std::vector<Eigen::VectorXf> &joint_configs,
                Eigen::MatrixXf &U, 
                Eigen::VectorXf &d){ 
            // do nothing
        };
        virtual void initialize() {
            // do nothing
        };
        
};