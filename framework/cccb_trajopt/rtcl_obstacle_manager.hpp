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
        virtual void computeCollisionConstraints(const std::vector<Eigen::VectorXf> &joint_configs,
                                Eigen::MatrixXf &U, Eigen::VectorXf &d);
        virtual void initialize();

        void mapObstacleCoeff(
            const Eigen::MatrixXf &U,
            const Eigen::MatrixXf &Ap,
            Eigen::MatrixXf &Actmp);

        void updateConstraintsCoeff(void* _debug_data,
                                    const uint num_joint_configs,
                                    const uint dim,
                                    Eigen::MatrixXf& U, 
                                    Eigen::VectorXf& d);

    protected:        
        rtcl::RtclInterface* rtcl_interface_;
};