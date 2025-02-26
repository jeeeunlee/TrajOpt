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

void RtclObstacleManager::updateObstacleCoeff(
        const std::vector<Eigen::VectorXf> &joint_configs,
        Eigen::MatrixXf & U, Eigen::VectorXf & d){
    // std::cout << " RtclObstacleManager::updateObstacleCoeff" << std::endl;
    // Clock localtimer;
    // localtimer.start();

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
            updateJointConfigsCoeff(&debug_data, num_joint_configs, dim, U, d);                
        }  
        

        // for (auto &q : joint_configs){
        //     const uint dim{q.size()};
        //     // std::cout << " start checkJointConfigCollisionDistance " << std::endl;
        //     robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(q);
        //     // localtimer.printElapsedMiliSec(" rtcl colission checker = ");
            
        //     rtcl::CollisionCheckerData debug_data;
        //     Ut = Eigen::MatrixXf::Zero(0, 0);
        //     dt = Eigen::VectorXf::Zero(0);
        //     if(rtcl_interface_->loadDebugData(debug_data)){
        //         // rtcl_interface_->saveDebugData(debug_data, q);                
        //         updateSingleJointCoeff(&debug_data, dim, Ut, dt);                
        //     }            
        //     // successfully loaded debug data from rtcl            
        //     U = rossy_utils::dStack(U, Ut);
        //     d = rossy_utils::vStack(d, dt);
        //     // localtimer.printElapsedMiliSec(" building constraints = ");
        // }
        // std::cout << " robot_collision_free = " << robot_collision_free << std::endl;
    }
}


void RtclObstacleManager::mapObstacleCoeff(
    const Eigen::MatrixXf & U, 
    const Eigen::MatrixXf & Ap, 
    Eigen::MatrixXf & Actmp){
    // Actmp = U*Ap_;
    rtcl_interface_->computeLongMatMul(
        U, Ap, Actmp);    
}

void RtclObstacleManager::updateJointConfigsCoeff(void* _debug_data,
                                                const uint num_joint_configs,
                                                const uint dim,
                                                Eigen::MatrixXf& U, 
                                                Eigen::VectorXf& d){
    // 
    rtcl::CollisionCheckerData* debug_data = 
        reinterpret_cast<rtcl::CollisionCheckerData*>(_debug_data);

    Eigen::MatrixXf Ucols = Eigen::MatrixXf::Zero(0,0);
    Eigen::VectorXf dcols = Eigen::VectorXf::Zero(0);

    uint ind_offset(0);
    for(uint ind_joint(0); ind_joint<num_joint_configs; ++ind_joint){
        const uint num_selected = debug_data->selected_num_per_configs[ind_joint];
        Eigen::MatrixXf Ut = debug_data->selected_ray_direction_projected.block(ind_offset, 0, num_selected, dim);        
        Eigen::VectorXf dt = debug_data->selected_distances.segment(ind_offset, num_selected);

        Ucols = rossy_utils::dStack(Ucols, Ut);
        dcols = rossy_utils::vStack(dcols, dt);

        ind_offset += num_selected;
        // Ut = Eigen::MatrixXf::Zero(num_constraints, dim);
        // dt = Eigen::VectorXf::Zero(num_constraints);
    }
    
    U = rossy_utils::vStack(U, Ucols);
    d = rossy_utils::vStack(d, dcols);


    // const uint num_constraints = (*debug_data->selected_indices).size();
    // Ut = Eigen::MatrixXf::Zero(num_constraints, dim);
    // dt = Eigen::VectorXf::Zero(num_constraints);

    // // select random indices         
    // for(int i(0); i<num_constraints; ++i){
    //     auto r_ind = (*debug_data->selected_indices)[i];
    //     // Ut.row(i) = (*debug_data.A_coeff)[r_ind];
    //     Ut.row(i) = (*debug_data->ray_direction_projected)[r_ind];
    //     dt(i) = (*debug_data->robot_points_distance_to_hit)(r_ind);
    // }
    
}
