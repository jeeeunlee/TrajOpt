#pragma once
// functions on cccb splines
#include "rossy_utils/io/io_utilities.hpp"

class ObstacleManager{
    public:
        ObstacleManager();
        ~ObstacleManager(){};

        void updateObstacleCoeff(Eigen::MatrixXd &U, Eigen::VectorXd &d);
};