#pragma once

#include "framework/obstacle_manager.hpp"
#include "framework/user_command.hpp"

namespace rtcl{
    class RtclInterface;
    // using CollisionCheckerData = ConstraintMultibodyRobotSingleJointConfigCollisionChecker::DebugData;
}


class RtclObstacleManager : public ObstacleManager {
    public:
        RtclObstacleManager(const std::string_view robot_name, 
                        const std::string_view assets_directory);
        ~RtclObstacleManager(){};
        
        // virtual void setObstacles(const std::vector<OBSTACLE>& obstacles);
        virtual void updateObstacleCoeff(const std::vector<Eigen::VectorXf> &joint_configs,
                                Eigen::MatrixXf &U, Eigen::VectorXf &d);

        void mapObstacleCoeff(
            const Eigen::MatrixXf &U,
            const Eigen::MatrixXf &Ap,
            Eigen::MatrixXf &Actmp);

        void updateSingleJointCoeff(void* _debug_data,
                                    const uint dim,
                                    Eigen::MatrixXf& Ut, 
                                    Eigen::VectorXf& dt);
        
        void updateRandomSingleJointCoeff(void* _debug_data,
                                        const uint num_constraints,
                                        const uint dim,
                                        Eigen::MatrixXf& Ut, 
                                        Eigen::VectorXf& dt);

    protected:        
        rtcl::RtclInterface* rtcl_interface_;
};