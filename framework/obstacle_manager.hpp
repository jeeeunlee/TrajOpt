#pragma once

#include "rossy_utils/io/io_utilities.hpp"
#include "framework/user_command.hpp"

#include <vector>

class ObstacleManager{
    public:
        ObstacleManager(){
            gripped_box_updated_ = false;
            obstacles_updated_ = false;
            b_initialized_ = false; // update rtcl given obstacles updated
        };
        virtual ~ObstacleManager(){}

        void setObstacles(const std::vector<OBSTACLE>& obstacles){
            obstacles_.clear();
            for (auto &obs: obstacles){
                // check
                // obs.printInfo();
                obstacles_.push_back(obs);        
            }
            obstacles_updated_ = true;
            b_initialized_ = false;
        };
        void setGrippedBox(const GRIPPED_BOX &gripped_box){
            // gripped_box.printInfo();
            gripped_box_ = gripped_box;
            gripped_box_updated_ = true;
            b_initialized_ = false;
        }
        virtual void computeCollisionConstraints(
            const std::vector<Eigen::VectorXf> &joint_configs,
            Eigen::MatrixXf &U, 
            Eigen::VectorXf &d) = 0;

        virtual void initialize() = 0;

    protected:
        std::vector<OBSTACLE> obstacles_;
        GRIPPED_BOX gripped_box_;
        bool gripped_box_updated_;
        bool obstacles_updated_;
        bool b_initialized_;

};