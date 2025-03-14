#include <gtest/gtest.h>

#include "rtcl/integrations/eigen/custom_typedefs.h"
#include "rtcl/wrapper/rtcl_interface.h"
#include "rtcl/tracers/collision_constraint_validation.h"
#include "framework/user_command.hpp"
#include "Configuration.h"
#include "test/json_utils.h"
#include <memory>


inline constexpr uint Dim = 8;
inline constexpr uint num_samples = pow(3,8)-1; // sampling on [-1,0,1] excluding {0,0..0}
inline constexpr uint trajId = 1; // trajectory id, can be either 1 or 5
constexpr std::array<float, 12> alpha_arr = {
    0.025f, 0.050f, 0.075f, 0.100f, 0.125f, 0.150f,
    0.175f, 0.200f, 0.225f, 0.250f, 0.275f, 0.300f
};

constexpr std::array<float, 6> alpha_arr_new = {
    0.05f, 0.10f, 0.15f, 0.20f, 0.25f, 0.30f  
};

class CollisionConstraintValidationTest : public ::testing::Test {
    public:
        std::unique_ptr<rtcl::RtclInterface> rtcl_interface_;

        std::vector<Eigen::VectorXf> joint_path_;
        std::vector<Eigen::VectorXf> constraints_vertices_;
        std::vector<Eigen::VectorXf> selected_collision_constraints_;
        std::vector<Eigen::VectorXf> simple_selected_collision_constraints_;
    private:

        nlohmann::json problem;

        rtcl::CollisionCheckerDataSingle debug_data;  
        rtcl::CollisionCheckerData debug_data_batch;

    public:
        CollisionConstraintValidationTest() {
            std::string robot_asset_dir = CURRENT_DIR "rtcl/assets";
            std::string robot_name_ = "ra830a";
            rtcl_interface_ = std::make_unique<rtcl::RtclInterface>(robot_name_, robot_asset_dir);
           
            std::cout<< " read file " << std::endl;
            problem = read_traj(robot_name_, trajId);
            set_obstacles(problem["obstacles"]);
            std::cout<< " set joint_path " << std::endl;
            json_listoflist_to_vecofeigen(problem["joint_path"], joint_path_, false); 

            // Since the number of collision constraints is 2 * Dim, we reserve the space for the selected collision constraints
            selected_collision_constraints_.reserve(joint_path_.size() * 2 * Dim);
        }

    void set_obstacles(const nlohmann::json &j_obs){
        std::cout<<"set_obstacles"<<std::endl;
        
        std::vector<Eigen::VectorXf> obstacle_poses_;
        std::vector<Eigen::Vector3f> obstacle_dimensions_;
        std::vector<Eigen::VectorXf> obstacle_mesh_poses_;
        std::vector<std::string> obstacle_mesh_paths_;
        
        const Eigen::VectorXf box_pose_from_ee_{{0.0f, 0.0f, 0.2032f, 1.0f, 0.0f, 0.0f, 0.0f}};
        const Eigen::Vector3f box_dimension_{{0.4064f, 0.4064f, 0.4064f}};

        for (auto& [key, val] : j_obs.items())
        {
            std::cout << "key: " << key << ", value:" << val << '\n';
            if (val["info"]["type"] == "box"){
                obstacle_poses_.push_back( json_list_to_eigen(val["pose"]) );
                obstacle_dimensions_.push_back(json_list_to_eigen(val["info"]["data"]));
            } else if (val["info"]["type"] == "mesh"){                
                std::string mesh_path = val["info"]["data"];
                // Append the prefix to the mesh path
                std::string assets_dir_ = CURRENT_DIR "test/testdata/furniture_task";
                std::string full_mesh_path = assets_dir_ + mesh_path;
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

    nlohmann::json read_traj(const std::string_view robotname, int trajNum){
        std::cout<<" read_traj " << std::endl;
        std::stringstream filenamess;
        filenamess << CURRENT_DIR  "test/testdata/furniture_task/" << robotname << "/case" << trajNum <<".json";
        std::string filename;
        filenamess >> filename;
        std::cout<<filename.c_str()<<std::endl;
        std::ifstream fJson(filename);
        std::stringstream buffer;
        buffer << fJson.rdbuf();      

        return nlohmann::json::parse(buffer.str());        
    }

    void updateCollisionConstraints(Eigen::VectorXf &joint_config){
        
        // compute collision constraints for single joint configuration with quickhull
        auto robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(joint_config);
        rtcl_interface_->loadDebugDataSingle(debug_data);

        std::vector<float>& tmp_constraints_vertices_ = debug_data.constraints_vertices;
        // Convert tmp_constraints_vertices_ into vector of Eigen::VectorXf
        constraints_vertices_.clear();
        constraints_vertices_.reserve(tmp_constraints_vertices_.size() / Dim);
        for (size_t i = 0; i < tmp_constraints_vertices_.size(); i += Dim) {
            Eigen::VectorXf constraint(Dim);
            for (size_t j = 0; j < Dim; j++) {
                constraint[j] = tmp_constraints_vertices_[i + j];
            }
            constraints_vertices_.push_back(constraint);
        }

        std::vector<uint>& selected_indices_ = debug_data.selected_indices;
        uint num_selected = debug_data.selected_num_per_configs[0];
        selected_collision_constraints_.resize(num_selected);
        for(uint i(0); i<num_selected; ++i){
            selected_collision_constraints_[i] = constraints_vertices_[selected_indices_[i]];
        }


        // compute collision constraints for single joint configuration with simple selection
        bool robot_collision_free2 = rtcl_interface_->checkJointConfigsCollisionDistance({joint_config});
        rtcl_interface_->loadDebugData(debug_data_batch);

        // In the same way, we process the simple selection
        std::vector<uint>& selected_indices_simple_ = debug_data_batch.selected_indices;
        uint num_simple_selected = debug_data_batch.selected_num_per_configs[0];
        simple_selected_collision_constraints_.resize(num_simple_selected);
        for(uint i(0); i<num_simple_selected; ++i){
            simple_selected_collision_constraints_[i] = constraints_vertices_[selected_indices_simple_[i]];
        }
    }  
};


TEST_F(CollisionConstraintValidationTest, DataCornerNormalized){

    std::vector<std::array<float, Dim>> poses_normalized;   
    std::vector<int> result;    
    rtcl::Eigen_::VectorXb bool_constraint;
    rtcl::Eigen_::VectorXb bool_selected_constraint;
    rtcl::Eigen_::VectorXb bool_ground_truth;

    poses_normalized.reserve(num_samples);
    result.reserve(num_samples);
    bool_constraint.resize(num_samples);
    bool_selected_constraint.resize(num_samples);
    bool_ground_truth.resize(num_samples);
    
    int poseId = 0;
    for (auto &joint_config: joint_path_){
        // update collision constraints for both quickhull and simple selection
        updateCollisionConstraints(joint_config);
        std::array<int,4> counts = {0,0,0,0};

            
        std::cout << "simple selection (Andrew's)"<<std::endl;
        std::cout << "alpha \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;

        for(auto &alpha: alpha_arr_new){
            genNormPose<Dim>(poses_normalized, alpha);
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                constraints_vertices_);

            rtcl::check_constraint(
                bool_selected_constraint,
                poses_normalized,
                simple_selected_collision_constraints_);

            rtcl::count_booleans(counts,
                bool_selected_constraint,
                bool_constraint );

            std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                    << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;
        }


        std::cout << "quick hull " << std::endl;
        std::cout << "alpha \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
        // quick hull
        for(auto &alpha: alpha_arr_new){
            genNormPose<Dim>(poses_normalized, alpha);
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                constraints_vertices_);
            rtcl::check_constraint(
                bool_selected_constraint,
                poses_normalized,
                simple_selected_collision_constraints_);

            rtcl::count_booleans(counts,
                bool_selected_constraint,
                bool_constraint );

            std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                    << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;
        }

        std::cout << "=================================" << std::endl;
        // CORNER NORMALIZED    
        // example : rss_RO_result_corner_normalized_0.030000.bin
        std::string result_dir_path = CURRENT_DIR "test/testdata/rt_result_sizhe_furniture/t" + std::to_string(trajId) + "/p" + std::to_string(poseId) + "/";    
        std::cout << "alpha \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
        for(auto &alpha: alpha_arr){
            // read Sizhe's result
            readBin(result_dir_path + "rss_RO_result_corner_normalized_" + std::to_string(alpha)+ ".bin", result);
            for (size_t i = 0; i < result.size(); ++i) {
                bool_ground_truth[i] = result[i]>0;
            }
            // gen samples
            genNormPose<Dim>(poses_normalized, alpha);
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                selected_collision_constraints_);
            rtcl::count_booleans(counts,
                bool_constraint,
                bool_ground_truth );

            std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                    << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;     

            // simple selection
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                simple_selected_collision_constraints_);
            rtcl::count_booleans(counts,
                bool_constraint,
                bool_ground_truth );

            std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                    << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;      
        }
    }
}

