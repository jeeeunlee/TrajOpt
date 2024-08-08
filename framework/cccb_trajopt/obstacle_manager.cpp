#include "obstacle_manager.hpp"
#include "framework/user_command.hpp"


ObstacleManager::ObstacleManager(RobotSystem* _robot)
{
    obstacles_.clear();    
}

void ObstacleManager::setObstacles(const std::vector<OBSTACLE>& obstacles){
    obstacles_.clear();
    for (auto obs: obstacles)
        obstacles_.push_back(&obs);
}   

void ObstacleManager::updateObstacleCoeff(
        const std::vector<Eigen::VectorXd> &joint_configs,
        Eigen::MatrixXd & U, Eigen::VectorXd & d){
    // update U and d
    
}