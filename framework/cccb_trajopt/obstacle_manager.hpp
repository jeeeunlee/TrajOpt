#pragma once
// functions on cccb splines
#include "rossy_utils/io/io_utilities.hpp"

class OBSTACLE;
class RobotSystem;

class ObstacleManager{
    public:
        ObstacleManager(RobotSystem* _robot);
        ~ObstacleManager(){};

        void setObstacles(const std::vector<OBSTACLE>& obstacles);
        void updateObstacleCoeff(const std::vector<Eigen::VectorXd> &joint_configs,
                                Eigen::MatrixXd &U, Eigen::VectorXd &d);
        

    public:
        std::vector<OBSTACLE*> obstacles_;
        RobotSystem* robot_;
};