#include "framework/cccb_trajopt/rtcl_obstacle_manager.hpp"
#include "framework/user_command.hpp"
#include "framework/user_command.hpp"
#include "rtcl/wrapper/rtcl_interface.h"
#include "rossy_utils/robot_system/robot_system.hpp"

RtclObstacleManager::RtclObstacleManager(const std::string_view robot_name, 
                                const std::string_view assets_directory)
                                : ObstacleManager() {
    rossy_utils::pretty_constructor(1, "RtclObstacleManager");
    obstacles_.clear();     
    rtcl_interface_ =  new rtcl::RtclInterface(robot_name, assets_directory);
}

void RtclObstacleManager::updateObstacleCoeff(
        const std::vector<Eigen::VectorXd> &joint_configs,
        Eigen::MatrixXd & U, Eigen::VectorXd & d){
    std::cout << " RtclObstacleManager::updateObstacleCoeff" << std::endl;

    if(obstacles_.size()>0){
        std::vector<Eigen::VectorXf> pose_list;
        std::vector<Eigen::Vector3f> dim_list;
        for(auto &obs: obstacles_){
            const Eigen::VectorXd pose{obs.pose};
            const Eigen::Vector3d dim{obs.dimension};
            std::cout << "pose = "<< pose.transpose() << std::endl;
            std::cout << "dim = "<< dim.transpose() << std::endl;
            pose_list.push_back(pose.cast<float>());
            dim_list.push_back(dim.cast<float>());            
        }
        rtcl_interface_->setBoxObstacles(pose_list, dim_list);
        

        // update U and d

        bool robot_collision_free = true;
        rtcl::CollisionCheckerData debug_data;
        for (auto &q : joint_configs){
            std::cout << " start checkJointConfigCollisionFreeWithDistance " << std::endl;
            robot_collision_free = rtcl_interface_->checkJointConfigCollisionFreeWithDistance(q.cast<float>());
            std::cout << " end checkJointConfigCollisionFreeWithDistance " << std::endl;
            if( rtcl_interface_->loadDebugData(debug_data) ){
                // successfully loaded debug data from rtcl
                std::cout << " debug data successfully loaded from rtcl " << std::endl;
            }
        }      
    }
    
}

// CollisionCheckerData{
// access_ptr<const std::vector<Eigen::Vector3f>> posed_robot_points;
// access_ptr<const std::vector<Eigen::Vector3f>> posed_robot_normals;
// access_ptr<const Eigen::VectorXf> robot_points_distance_to_hit;    
// access_ptr<const Eigen_::VectorXb> robot_points_collision_free;
// access_ptr<const Eigen_::VectorXb> robot_points_selected; // ignore the one with big distances
// access_ptr<const std::vector<size_t>> num_of_points_each_link; }