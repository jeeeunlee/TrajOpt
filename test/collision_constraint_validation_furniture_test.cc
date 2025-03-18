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
inline constexpr uint trajId = 0; // trajectory id, can be either 0 or 1
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
        std::vector<float> ray_dir_projected_; //[Nc][dim]
        std::vector<float> distance_; //[Nc]
        std::vector<float> selected_ray_dir_projected_;
        std::vector<float> selected_distance_;
        std::vector<float> simple_selected_ray_dir_projected_;
        std::vector<float> simple_selected_distance_;
        std::vector<uint> selected_indices_;
        std::vector<uint> selected_indices_simple_;
    private:

        nlohmann::json problem;

        rtcl::CollisionCheckerDataSingle debug_data_;  
        rtcl::CollisionCheckerData debug_data_batch_;

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
            std::cout << " joint_path = "<< joint_path_.size() << std::endl;
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

        uint dim = joint_config.size();
        
        // compute collision constraints for single joint configuration with quickhull
        auto robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(joint_config);
        rtcl_interface_->loadDebugDataSingle(debug_data_);
        ray_dir_projected_ = debug_data_.ray_dir_projected;
        distance_ = debug_data_.distance;
        selected_indices_ = debug_data_.selected_indices;
        uint num_selected = debug_data_.selected_num_per_configs[0];
        selected_ray_dir_projected_.resize(num_selected*dim);
        Eigen::MatrixXf tmp = debug_data_.selected_ray_direction_projected.transpose();
        std::copy(tmp.data(), 
                tmp.data() + num_selected*dim, 
                selected_ray_dir_projected_.begin());
        selected_distance_.resize(num_selected);
        std::copy(debug_data_.selected_distances.data(), 
                debug_data_.selected_distances.data() + num_selected, 
                selected_distance_.begin());

        //check
        // int check_select_id(3);
        // for(int idim(0); idim < dim; ++idim)
        //     std::cout << selected_ray_dir_projected_[check_select_id*dim + idim] << ", ";
        // std::cout << std::endl;
        // std::cout << debug_data_.selected_ray_direction_projected.row(check_select_id) << std::endl;        
        // for(int idim(0); idim < dim; ++idim)
        //     std::cout << ray_dir_projected_[selected_indices_[check_select_id]*dim + idim] << ", ";
        // std::cout << std::endl;
        // std::cout << selected_distance_[check_select_id] << "=" << distance_[selected_indices_[check_select_id]] << std::endl;

        // compute collision constraints for single joint configuration with simple selection
        bool robot_collision_free2 = rtcl_interface_->checkJointConfigsCollisionDistance({joint_config});
            rtcl_interface_->loadDebugData(debug_data_batch_);

        // In the same way, we process the simple selection
        selected_indices_simple_ = debug_data_batch_.selected_indices;
        uint num_simple_selected = debug_data_batch_.selected_num_per_configs[0];
        simple_selected_ray_dir_projected_.resize(num_simple_selected*dim);
        tmp = debug_data_batch_.selected_ray_direction_projected.transpose();
        std::copy(tmp.data(), 
                tmp.data() + num_simple_selected*dim, 
                simple_selected_ray_dir_projected_.begin());
        simple_selected_distance_.resize(num_simple_selected);
        std::copy(debug_data_batch_.selected_distances.data(), 
                debug_data_batch_.selected_distances.data() + num_simple_selected, 
                simple_selected_distance_.begin());

        // for(int idim(0); idim < dim; ++idim)
        //     std::cout << simple_selected_ray_dir_projected_[check_select_id*dim + idim] << ", ";
        // std::cout << std::endl;
        // std::cout << debug_data_batch_.selected_ray_direction_projected.row(check_select_id) << std::endl;
        // for(int idim(0); idim < dim; ++idim)
        //     std::cout << ray_dir_projected_[selected_indices_simple_[check_select_id]*dim + idim] << ", ";
        // std::cout << std::endl;
        // std::cout << simple_selected_distance_[check_select_id] << "=" << distance_[selected_indices_simple_[check_select_id]] << std::endl;
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

    std::vector<std::array<int,4>> counts_simple_selection_vs_all = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    std::vector<std::array<int,4>> counts_quickhull_vs_all = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    std::vector<std::array<int,4>> counts_simple_selection_vs_ground_truth = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    std::vector<std::array<int,4>> counts_quickhull_vs_ground_truth = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    std::vector<std::array<int,4>> counts_all_vs_ground_truth = {
        {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
    
    int poseId = 0;
    for (int poseId(0); poseId< joint_path_.size(); poseId++){
        auto joint_config = joint_path_[poseId];
        std::cout << poseId << "th joint " << std::endl;
        // update collision constraints for both quickhull and simple selection
        updateCollisionConstraints(joint_config);
        std::array<int,4> counts = {0,0,0,0};

        // std::cout << "simple selection (Andrew's)(" << simple_selected_distance_.size() << ") : ";
        // for(auto &ind: selected_indices_simple_) std::cout << ind << ", ";
        // std::cout<<std::endl;
        // std::cout << "alpha \t FF(TrueNegative) FT(FalseNegative) \t TT(TruePositive) TF(FalsePositive)"<<std::endl;
        int i_alpha = 0;
        for(auto &alpha: alpha_arr_new){
            genNormPose<Dim>(poses_normalized, alpha);
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                ray_dir_projected_,
                distance_);

            rtcl::check_constraint(
                bool_selected_constraint,
                poses_normalized,
                simple_selected_ray_dir_projected_,
                simple_selected_distance_);

            rtcl::count_booleans(counts,
                bool_selected_constraint,
                bool_constraint );

            // std::cout << alpha << " " << counts[0] << " " << counts[1] << " " 
            //         << counts[2] << " " << counts[3] << " " << std::endl;

            for(int k = 0; k < 4; k++) counts_simple_selection_vs_all[i_alpha][k] += counts[k];
            i_alpha++;
        }


        // std::cout << "quick hull(" << selected_distance_.size() << ") : ";
        // for(auto &ind: selected_indices_) std::cout << ind << ", ";
        // std::cout<<std::endl;
        // std::cout << "alpha \t FF(TrueNegative) FT(FalseNegative) \t TT(TruePositive) TF(FalsePositive)"<<std::endl;
        // quick hull
        i_alpha = 0;
        for(auto &alpha: alpha_arr_new){
            genNormPose<Dim>(poses_normalized, alpha);
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                ray_dir_projected_,
                distance_);
            rtcl::check_constraint(
                bool_selected_constraint,
                poses_normalized,
                selected_ray_dir_projected_,
                selected_distance_);

            rtcl::count_booleans(counts,
                bool_selected_constraint,
                bool_constraint );

            // std::cout << alpha << " " << counts[0] << " " << counts[1] << " " 
            //         << counts[2] << " " << counts[3] << " " << std::endl;

            for(int k = 0; k < 4; k++) counts_quickhull_vs_all[i_alpha][k] += counts[k];
            i_alpha++;
        }

        // std::cout << "" << std::endl;
        // CORNER NORMALIZED    
        // example : rss_RO_result_corner_normalized_0.030000.bin
        std::string result_dir_path = CURRENT_DIR "test/testdata/rt_result_sizhe_furniture/t" + std::to_string(trajId) + "/p" + std::to_string(poseId) + "/";    
        // std::cout << "alpha \t FF(TrueNegative) FT(FalseNegative) \t TT(TruePositive) TF(FalsePositive)"<<std::endl;
        i_alpha = 0;
        for(auto &alpha: alpha_arr_new){
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
                selected_ray_dir_projected_,
                selected_distance_);
            rtcl::count_booleans(counts,
                bool_constraint,
                bool_ground_truth );
            for(int k = 0; k < 4; k++) counts_quickhull_vs_ground_truth[i_alpha][k] += counts[k];

            // std::cout << alpha << " (qh)\t" << counts[0] << " " << counts[1] << " " 
            //         << counts[2] << " " << counts[3] << " " << std::endl;     

            // simple selection
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                simple_selected_ray_dir_projected_,
                simple_selected_distance_);
            rtcl::count_booleans(counts,
                bool_constraint,
                bool_ground_truth );
            for(int k = 0; k < 4; k++) counts_simple_selection_vs_ground_truth[i_alpha][k] += counts[k];

            // std::cout << alpha << " (sp)\t" << counts[0] << " " << counts[1] << " " 
            //         << counts[2] << " " << counts[3] << " " << std::endl;      

            // all
            rtcl::check_constraint(
                bool_constraint,
                poses_normalized,
                ray_dir_projected_,
                distance_);
            rtcl::count_booleans(counts,
                bool_constraint,
                bool_ground_truth );
            for(int k = 0; k < 4; k++) counts_all_vs_ground_truth[i_alpha][k] += counts[k];

            
            i_alpha++;
        }
    }

    std::cout << " simple_selection_vs_all =================================" << std::endl;
    std::cout << "alpha FF(TrueNegative) FT(FalseNegative) TT(TruePositive) TF(FalsePositive)"<<std::endl;
    for(int i = 0; i < alpha_arr_new.size(); i++){
        std::cout << alpha_arr_new[i] << " " << counts_simple_selection_vs_all[i][0] << " " << counts_simple_selection_vs_all[i][1] << " " 
                << counts_simple_selection_vs_all[i][2] << " " << counts_simple_selection_vs_all[i][3] << " " << std::endl;
    }
    std::cout << " quickhull_vs_all =================================" << std::endl;
    std::cout << "alpha FF(TrueNegative) FT(FalseNegative) TT(TruePositive) TF(FalsePositive)"<<std::endl;
    for(int i = 0; i < alpha_arr_new.size(); i++){
        std::cout << alpha_arr_new[i] << " " << counts_quickhull_vs_all[i][0] << " " << counts_quickhull_vs_all[i][1] << " " 
                << counts_quickhull_vs_all[i][2] << " " << counts_quickhull_vs_all[i][3] << " " << std::endl;
    }   
    std::cout << " simple_selection_vs_ground_truth =================================" << std::endl;
    std::cout << "alpha FF(TrueNegative) FT(FalseNegative) TT(TruePositive) TF(FalsePositive)"<<std::endl;
    for(int i = 0; i < alpha_arr_new.size(); i++){
        std::cout << alpha_arr_new[i] << " " << counts_simple_selection_vs_ground_truth[i][0] << " " << counts_simple_selection_vs_ground_truth[i][1] << " " 
                << counts_simple_selection_vs_ground_truth[i][2] << " " << counts_simple_selection_vs_ground_truth[i][3] << " " << std::endl;
    }       
    std::cout << " quickhull_vs_ground_truth =================================" << std::endl;
    std::cout << "alpha FF(TrueNegative) FT(FalseNegative) TT(TruePositive) TF(FalsePositive)"<<std::endl;
    for(int i = 0; i < alpha_arr_new.size(); i++){
        std::cout << alpha_arr_new[i] << " " << counts_quickhull_vs_ground_truth[i][0] << " " << counts_quickhull_vs_ground_truth[i][1] << " " 
                << counts_quickhull_vs_ground_truth[i][2] << " " << counts_quickhull_vs_ground_truth[i][3] << " " << std::endl;
    }      
    std::cout << " all_vs_ground_truth =================================" << std::endl;
    std::cout << "alpha FF(TrueNegative) FT(FalseNegative) TT(TruePositive) TF(FalsePositive)"<<std::endl;
    for(int i = 0; i < alpha_arr_new.size(); i++){
        std::cout << alpha_arr_new[i] << " " << counts_all_vs_ground_truth[i][0] << " " << counts_all_vs_ground_truth[i][1] << " " 
                << counts_all_vs_ground_truth[i][2] << " " << counts_all_vs_ground_truth[i][3] << " " << std::endl;
    }
    
    
}

