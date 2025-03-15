#include "framework/cccb_trajopt/rtcl_obstacle_manager.hpp"
#include "framework/user_command.hpp"
#include "framework/user_command.hpp"

#include "rossy_utils/robot_system/robot_system.hpp"
#include "rossy_utils/math/math_utilities.hpp"
// for benchmark
#include "rossy_utils/general/clock.hpp"
#include "rtcl_obstacle_manager.hpp"
#include "rtcl/wrapper/rtcl_interface.h"

RtclObstacleManager::RtclObstacleManager(const std::string_view robot_name, 
                                const std::string_view assets_directory)
                                : ObstacleManager() {
    rossy_utils::pretty_constructor(1, "RtclObstacleManager");
    obstacles_.clear();     
    rtcl_interface_ =  new rtcl::RtclInterface(robot_name, assets_directory);
}

void RtclObstacleManager::initialize(){
    if(!b_initialized_){
        b_initialized_ = true;
        // set gripped box
        if(gripped_box_updated_) {
            rtcl_interface_->clearGrippedBox();
            rtcl_interface_->setGrippedBox(gripped_box_.pose_from_ee, 
                                        gripped_box_.dimension);
            gripped_box_updated_ = false;
        }
        // localtimer.printElapsedMiliSec(" set gripped box = ");

        // set obstacles
        if(obstacles_updated_) {        
            std::vector<Eigen::VectorXf> pose_list; // box
            std::vector<Eigen::Vector3f> dim_list; // box
            std::vector<Eigen::VectorXf> pose_list_mesh; // mesh
            std::vector<std::string> mesh_path_list; // mesh

            auto box_view = obstacles_ | std::views::filter([](const OBSTACLE &obs) {
                return obs.type == 0;
            });

            for(auto &obs: box_view){
                const Eigen::VectorXf pose{obs.pose};
                const Eigen::Vector3f dim{obs.dimension};
                // std::cout << "pose = "<< pose.transpose() << std::endl;
                // std::cout << "dim = "<< dim.transpose() << std::endl;
                pose_list.push_back(pose);
                dim_list.push_back(dim);            
            }
            rtcl_interface_->clearBoxObstacles();
            rtcl_interface_->setBoxObstacles(pose_list, dim_list);

            auto mesh_view = obstacles_ | std::views::filter([](const OBSTACLE &obs) {
                return obs.type == 1;
            });


            for(auto &obs: mesh_view){
                pose_list_mesh.push_back(obs.pose);
                mesh_path_list.push_back(obs.meshPath);
            }
            rtcl_interface_->clearMeshObstacles();
            rtcl_interface_->setMeshObstacles(pose_list_mesh, mesh_path_list);

            obstacles_updated_ = false;
        }        
    }

    // initialize optix once by running raytracing once: 
    // later: make function only do launch optix for initializing
    static bool optix_initialized = false;
    if(!optix_initialized){
        bool robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance();
        optix_initialized = true;
    }
    
}


void RtclObstacleManager::computeCollisionConstraints(
        const std::vector<Eigen::VectorXf> &joint_configs,
        Eigen::MatrixXf & U, Eigen::VectorXf & d){
    // std::cout << " RtclObstacleManager::computeCollisionConstraints" << std::endl;
    // Clock localtimer;
    // localtimer.start();

    // update gripped box and obstacles in rtcl
    initialize();
    // localtimer.printElapsedMiliSec(" set obstacles = ");

    if(obstacles_.size()>0 && joint_configs.size()>0)
    {
        // update U and d
        bool robot_collision_free = true;

        robot_collision_free = rtcl_interface_->checkJointConfigsCollisionDistance(joint_configs);
        const uint num_joint_configs{joint_configs.size()};
        const uint dim{joint_configs[0].size()};
        
        rtcl::CollisionCheckerData debug_data;
        if(rtcl_interface_->loadDebugData(debug_data)){
            // rtcl_interface_->saveDebugData(debug_data, q);                
            updateConstraintsCoeff(&debug_data, num_joint_configs, dim, U, d);                
        }  
    }
}


void RtclObstacleManager::mapObstacleCoeff(
    const Eigen::MatrixXf & U, 
    const Eigen::MatrixXf & Ap, 
    Eigen::MatrixXf & Actmp){
    // Actmp = U*Ap_;
    rtcl_interface_->computeLargeMatMul(
        U, Ap, Actmp);    
}

void RtclObstacleManager::updateConstraintsCoeff(void* _debug_data,
                                                const uint num_joint_configs,
                                                const uint dim,
                                                Eigen::MatrixXf& U, 
                                                Eigen::VectorXf& d){
    // 
    rtcl::CollisionCheckerData* debug_data = 
        reinterpret_cast<rtcl::CollisionCheckerData*>(_debug_data);

    uint num_constraints = debug_data->selected_distances.size();
    U = Eigen::MatrixXf::Zero(num_constraints, dim*num_joint_configs);
    d = Eigen::VectorXf::Zero(num_constraints);

    uint ind_offset(0);
    for(uint ind_joint(0); ind_joint<num_joint_configs; ++ind_joint){
        const uint num_selected = debug_data->selected_num_per_configs[ind_joint];
        const Eigen::MatrixXf &Ut = debug_data->selected_ray_direction_projected.block(ind_offset, 0, num_selected, dim);        
        const Eigen::VectorXf &dt = debug_data->selected_distances.segment(ind_offset, num_selected);
        const float relaxed_coeff = (float)ind_joint * ((float)num_joint_configs-1.f-(float)ind_joint) 
        / ( ((float)num_joint_configs-1.f)*((float)num_joint_configs-1.f) );
        const Eigen::VectorXf &dt_relaxed = Eigen::VectorXf::Constant(num_selected, 0.4f*relaxed_coeff*relaxed_coeff);
        // std::cout << "dt_relaxed = " << 1.f*relaxed_coeff*relaxed_coeff << std::endl;
        U.block(ind_offset, ind_joint*dim, num_selected, dim) = Ut;
        d.segment(ind_offset, num_selected) = dt-dt_relaxed;

        ind_offset += num_selected;
    }

    
}
