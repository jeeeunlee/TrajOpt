#include <gtest/gtest.h>

#include "rtcl/integrations/eigen/custom_typedefs.h"
#include "rtcl/wrapper/rtcl_interface.h"
#include "rtcl/tracers/collision_constraint_validation.h"
#include "framework/user_command.hpp"
#include "test/json_utils.h"
#include "Configuration.h"


class CollisionConstraintSelectionTest : public ::testing::Test {
    public:
        CollisionConstraintSelectionTest()
            :assets_dir_(CURRENT_DIR "rtcl/assets"), robot_name_("ra830a") {
            rtcl_interface_ = new rtcl::RtclInterface(robot_name_, assets_dir_);
        }

    nlohmann::json read_case(const std::string_view robotname, int casenum){
        std::cout<<" read_case " << std::endl;
        std::stringstream filenamess;
        filenamess << CURRENT_DIR "test/testdata/furniture_task/" << robotname << "/case" << casenum <<".json";
        std::string filename;
        filenamess >> filename;
        std::cout<<filename.c_str()<<std::endl;
        std::ifstream fJson(filename);
        std::stringstream buffer;
        buffer << fJson.rdbuf();      

        return nlohmann::json::parse(buffer.str());        
    }

    void set_obstacles(const nlohmann::json &j_obs){
        std::cout<<"set_obstacles"<<std::endl;
        for (auto& [key, val] : j_obs.items())
        {
            std::cout << "key: " << key << ", value:" << val << '\n';
            if (val["info"]["type"] == "box"){
                obstacle_poses_.push_back( json_list_to_eigen(val["pose"]) );
                obstacle_dimensions_.push_back(json_list_to_eigen(val["info"]["data"]));
            } else if (val["info"]["type"] == "mesh"){                
                std::string mesh_path = val["info"]["data"];
                // Append the prefix to the mesh path
                std::string assets_dir = CURRENT_DIR "test/testdata/furniture_task";
                std::string full_mesh_path = assets_dir + mesh_path;
                obstacle_mesh_paths_.push_back(full_mesh_path);
                obstacle_mesh_poses_.push_back( json_list_to_eigen(val["pose"]) );                
            }
        }

        // set obstacles
        rtcl_interface_->clearBoxObstacles();
        rtcl_interface_->setBoxObstacles(obstacle_poses_, obstacle_dimensions_);

        rtcl_interface_->clearMeshObstacles();
        rtcl_interface_->setMeshObstacles(obstacle_mesh_poses_, obstacle_mesh_paths_);

        // set gripped box
        rtcl_interface_->clearGrippedBox();
        rtcl_interface_->setGrippedBox(box_pose_from_ee_, box_dimension_);


    }

    void checkSingleCollisionConstraints(const rtcl::CollisionCheckerDataSingle& debug_data){        
        std::vector<uint> selected_indices(debug_data.selected_indices);

        std::cout << "selected_indices single = " << std::endl;
        for(auto &id: selected_indices){
            std::cout << id <<", ";
        }
        std::cout << std::endl;

        // std::vector<Eigen::VectorXf> ray_dir_projected = *(debug_data.ray_direction_projected);
        // Eigen::VectorXf dist_to_hit = *(debug_data.robot_points_distance_to_hit);
    }

    void checkBatchCollisionConstraints(const rtcl::CollisionCheckerData& debug_data){        
        const std::vector<uint> &selected_indices(debug_data.selected_indices);
        const std::vector<uint> &selected_num_per_configs(debug_data.selected_num_per_configs);

        std::cout << "selected_num_per_configs = " << std::endl;
        for(auto &num: selected_num_per_configs){
            std::cout << num <<", ";
        }
        std::cout << std::endl;

        // std::cout << "selected_indices batch = " << std::endl;
        // for(auto &id: selected_indices){
        //     std::cout << id <<", ";
        // }
        // std::cout << std::endl;

        std::cout << "selected_indices from base = " << std::endl;
        uint ind_base(0);
        for(uint i(0); i<selected_num_per_configs.size(); ++i){
            uint n = selected_num_per_configs[i];
            for(uint j(ind_base); j<ind_base+n; ++j){
                std::cout << selected_indices[j] - i*22162 <<", ";
            }        
            std::cout << std::endl;    
            ind_base += n;
        }        
    }    
    
    public:
        std::string assets_dir_;
        std::string robot_name_;

        std::vector<Eigen::VectorXf> joint_path_;
        const Eigen::VectorXf box_pose_from_ee_{{0.0f, 0.0f, 0.2032f, 1.0f, 0.0f, 0.0f, 0.0f}};
        const Eigen::Vector3f box_dimension_{{0.4064f, 0.4064f, 0.4064f}};
        std::vector<Eigen::VectorXf> obstacle_poses_;
        std::vector<Eigen::Vector3f> obstacle_dimensions_;
        std::vector<Eigen::VectorXf> obstacle_mesh_poses_;
        std::vector<std::string> obstacle_mesh_paths_;

        rtcl::RtclInterface* rtcl_interface_;
          
};


TEST_F(CollisionConstraintSelectionTest, QuickhullTest){

    std::cout<< " read file " << std::endl;
    auto problem = read_case(robot_name_,1);
    std::cout<< " set_obstacles " << std::endl;
    set_obstacles(problem["obstacles"]);
    std::cout<< " set joint_path " << std::endl;
    json_listoflist_to_vecofeigen(problem["joint_path"], joint_path_, false);    
    
    const uint Dim = joint_path_[0].size();

    // compute collision constraints for single joint configuration
    std::cout<< " compute single joint 0" << std::endl;
    bool robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(joint_path_[0]);  
    rtcl::CollisionCheckerDataSingle debug_data;  
    rtcl_interface_->loadDebugDataSingle(debug_data);
    checkSingleCollisionConstraints(debug_data);

    // compute collision constraints for single joint configuration
    std::cout<< " compute single joint 1" << std::endl;
    robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(joint_path_[1]);  
    rtcl_interface_->loadDebugDataSingle(debug_data);
    checkSingleCollisionConstraints(debug_data);

    std::cout<< " compute batch joint [0,1]" << std::endl;
    // bool robot_collision_free_path = rtcl_interface_->checkJointConfigsCollisionDistance(joint_path_);
    bool robot_collision_free_path = rtcl_interface_->checkJointConfigsCollisionDistance({joint_path_[0], joint_path_[1]});
    rtcl::CollisionCheckerData debug_data_batch;
    rtcl_interface_->loadDebugData(debug_data_batch);
    checkBatchCollisionConstraints(debug_data_batch);
    

}


