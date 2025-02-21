#include <gtest/gtest.h>

#include "rtcl/integrations/eigen/custom_typedefs.h"
#include "rtcl/wrapper/rtcl_interface.h"
#include "rtcl/tracers/collision_constraint_validation.h"
#include "framework/user_command.hpp"
#include "Configuration.h"


class CollisionConstraintValidationTest : public ::testing::Test {
    public:
        CollisionConstraintValidationTest(){
            std::string assets_dir = CURRENT_DIR "rtcl/assets";
            std::string robot_name = "ra830a";
            rtcl_interface_ = new rtcl::RtclInterface(robot_name, assets_dir);
        }

    void read_case(const uint folder_num, 
                Eigen::VectorXf& joint_config,
                std::vector<Eigen::VectorXf> &obstacle_poses,
                std::vector<Eigen::Vector3f> &obstacle_dimensions){
        // std::cout <<" read info " << std::endl;
        std::ostringstream data_path;
        data_path << LOG_DIR;
        data_path << "/d" << folder_num << "/"; 

        std::ifstream myfile;
        myfile.open(data_path.str() +"joint_config.txt");
        std::string line;
        if(std::getline(myfile, line)){
            std::stringstream ss(line);
            std::vector<float> values;
            float value;
            while (ss >> value) {
                values.push_back(value);
            }             
            joint_config = Eigen::Map<Eigen::VectorXf>(values.data(), values.size());
        }
        myfile.close();

        obstacle_poses.clear();
        obstacle_dimensions.clear();
        myfile.open(data_path.str() +"boxes.txt");
        while (std::getline(myfile, line)) {
            std::stringstream ss(line);
            std::vector<float> values;
            float value;
            // Parse the line into individual float values
            while (ss >> value) {
                values.push_back(value);
            }
            Eigen::VectorXf pose {{values[0], values[1], values[2], 1., 0., 0., 0.}};
            Eigen::Vector3f dim {{values[3], values[4], values[5]}};
            dim *= 2.f; // dim = halfdim * 2
            obstacle_poses.push_back(pose);
            obstacle_dimensions.push_back(dim); 

        }
        myfile.close();
    }

    
    template <size_t ARRAY_SIZE>
    void read_simple_selection(const uint folder_num, 
                std::array<int, ARRAY_SIZE> &data){
        std::ostringstream data_path;
        data_path << CURRENT_DIR "test/testdata/rt-result-sizhe/simple_selected/simple_selected_";
        data_path << folder_num << ".txt"; 

        std::ifstream file(data_path.str());
        if (!file) {
            std::cerr << "Error: Could not open file " << data_path.str() << std::endl;
            return;
        }
        int value, index = 0;
        while (file >> value && index < ARRAY_SIZE) {            
            data[index++] = value;
        }
        if (index < ARRAY_SIZE) {
            std::cerr << "Warning: File contains fewer than " << ARRAY_SIZE << " values." << std::endl;
        }        
    }

    void update_collision_constraints(){
        // set obstacles
        rtcl_interface_->clearBoxObstacles();
        rtcl_interface_->setBoxObstacles(obstacle_poses_, obstacle_dimensions_);

        // set gripped box
        rtcl_interface_->clearGrippedBox();
        rtcl_interface_->setGrippedBox(box_pose_from_ee_, box_dimension_);

        // compute collision constraints for single joint configuration
        bool robot_collision_free = rtcl_interface_->checkJointConfigCollisionDistance(joint_config_);
        rtcl_interface_->loadDebugDataSingle(debug_data_);
        updateCollisionConstraints(debug_data_);
    }

    void updateCollisionConstraints(const rtcl::CollisionCheckerDataSingle& debug_data){        
        std::vector<Eigen::VectorXf> ray_dir_projected 
            = *(debug_data.ray_direction_projected);
        Eigen::VectorXf dist_to_hit 
            = *(debug_data.robot_points_distance_to_hit);
        std::vector<uint> selected_indices
            = *(debug_data.selected_indices);

        collision_constraints_.clear();
        for(int i(0); i<ray_dir_projected.size(); ++i){
            collision_constraints_.push_back(ray_dir_projected[i]/dist_to_hit(i));
        }

        selected_collision_constraints_.clear();
        // std::cout << "selected from quickhull = ";
        for(auto &ind: selected_indices){
            // std::cout << ind << ", ";
            selected_collision_constraints_.push_back(collision_constraints_[ind]);
        }        
        // std::cout << std::endl;
    }
        
    
    public:
        Eigen::VectorXf joint_config_;
        const Eigen::VectorXf box_pose_from_ee_{{0.0f, 0.0f, 0.2032f, 1.0f, 0.0f, 0.0f, 0.0f}};
        const Eigen::Vector3f box_dimension_{{0.4064f, 0.4064f, 0.4064f}};
        std::vector<Eigen::VectorXf> obstacle_poses_;
        std::vector<Eigen::Vector3f> obstacle_dimensions_;

        std::vector<Eigen::VectorXf> collision_constraints_;
        std::vector<Eigen::VectorXf> selected_collision_constraints_;

        rtcl::RtclInterface* rtcl_interface_;
        rtcl::CollisionCheckerDataSingle debug_data_;    
};



// TEST_F(CollisionConstraintValidationTest, SamplePoints){
//     // read cases
//     const uint folder_num = 21;     
//     read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

//     const float alpha{0.1f};
//     const uint num_interval{10};
//     // rtcl::sample_points_example(joint_config_, alpha, num_interval);
//     // std::cout <<"========" << std::endl;
//     // rtcl::sample_points_example2(joint_config_, alpha, num_interval);
//     // std::cout <<"========" << std::endl;
//     rtcl::sample_points_example3<8>(joint_config_, alpha);
//     std::cout <<"========" << std::endl;
// }


// TEST_F(CollisionConstraintValidationTest, BoxesObstacles){
    
//     // read cases
//     const uint folder_num = 21;     
//     read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

//     // update collision_constraints_, selected_collision_constraints_
//     update_collision_constraints();

//     const float alpha{0.1f};
//     const uint num_interval{10};
//     std::cout << "alpha="<<alpha << ", num_interval="<<num_interval<<std::endl;
//     std::cout << "\t\t FF \t FT \t TT \t TF"<<std::endl;
//     rtcl::quickhull_validation(joint_config_, 
//                         collision_constraints_, 
//                         selected_collision_constraints_,
//                         alpha, num_interval);
// }

// TEST_F(CollisionConstraintValidationTest, SizheData){
    
//     // read cases
//     const uint folder_num = 21;     
//     read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

//     // update collision_constraints_, selected_collision_constraints_
//     update_collision_constraints();

//     float alpha{0.075f};
//     uint num_interval{10};
//     std::cout << "alpha="<<alpha << ", num_interval="<<num_interval<<std::endl;
//     std::cout << "\t\t FF \t FT \t TT \t TF"<<std::endl;
//     rtcl::quickhull_validation(joint_config_, 
//                         collision_constraints_, 
//                         selected_collision_constraints_,
//                         alpha, num_interval);

//     // std::string result_dir_path = CURRENT_DIR "test/testdata/rt-result-sizhe/pose_21/";?=
//     std::string result_dir_path = CURRENT_DIR "test/testdata/rt-result-sizhe/pose_21_0075/";
//     // std::string result_dir_path = CURRENT_DIR "test/testdata/rt-result-sizhe/pose_21_005/";
//     std::cout << "\t\t FF \t FT \t TT \t TF"<<std::endl;
//     rtcl::collision_constraint_validation(result_dir_path,
//                         joint_config_, 
//                         selected_collision_constraints_);

// }

// TEST_F(CollisionConstraintValidationTest, DataCorner){
    
//     // read cases
//     const uint Dim = 8;
//     const uint folder_num = 21;     
//     read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

//     // update collision_constraints_, selected_collision_constraints_
//     update_collision_constraints();            
    
//     const uint num_samples = pow(3,8)-1; // sampling on [-1,0,1] excluding {0,0..0}
//     std::vector<int> result;
//     result.reserve(num_samples);

//     rtcl::Eigen_::VectorXb bool_constraint;
//     rtcl::Eigen_::VectorXb bool_ground_truth;
//     bool_ground_truth.resize(num_samples);
//     std::array<int,4> counts = {0,0,0,0};
//     std::vector<std::array<float, Dim>> poses_corner;
    

//     // Ground Truth: Sizhe's collision check data
//     // example : rss_RO_result_corner_0.015000.bin
//     std::string result_dir_path = CURRENT_DIR "test/testdata/rt-result-sizhe/corner/";    
//     std::array<float, 12> alpha_arr = {0.015f, 0.03f, 0.045f, 0.06f, 
//         0.075f, 0.09f, 0.105f, 0.12f, 0.135f, 0.15f, 0.165f, 0.18f};
//     // std::array<float, 1> alpha_arr = {0.015f};
//     for(auto &alpha: alpha_arr){
//         // read Sizhe's result
//         readBin(result_dir_path + "rss_RO_result_corner_" + std::to_string(alpha)+ ".bin", result);
//         for (size_t i = 0; i < result.size(); ++i) {
//             bool_ground_truth[i] = result[i]>0;
//         }
//         // gen samples
//         genPose<Dim>(poses_corner, alpha);

//         // check constraints
//         rtcl::check_constraint(
//             bool_constraint,
//             poses_corner,
//             collision_constraints_);

//         rtcl::count_booleans(counts,
//             bool_constraint,
//             bool_ground_truth );
//         std::cout << "alpha=" << alpha << std::endl;
//         std::cout << "FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
//         std::cout << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
//                   << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;
//     }       
// }

TEST_F(CollisionConstraintValidationTest, DataCornerNormalized){
    
    // read cases
    const uint Dim = 8;
    const uint folder_num = 21;     
    read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

    // update collision_constraints_, selected_collision_constraints_
    update_collision_constraints();   
    
    const uint num_samples = pow(3,8)-1; // sampling on [-1,0,1] excluding {0,0..0}

    std::vector<std::array<float, Dim>> poses_noramlized;   
    std::vector<int> result;    
    rtcl::Eigen_::VectorXb bool_constraint;
    rtcl::Eigen_::VectorXb bool_selected_constraint;
    rtcl::Eigen_::VectorXb bool_ground_truth;

    poses_noramlized.reserve(num_samples);
    result.reserve(num_samples);
    bool_constraint.resize(num_samples);
    bool_selected_constraint.resize(num_samples);
    bool_ground_truth.resize(num_samples);
           
    std::array<int,4> counts = {0,0,0,0};
    std::array<float, 12> alpha_arr = {0.015f, 0.03f, 0.045f, 0.06f, 
        0.075f, 0.09f, 0.105f, 0.12f, 0.135f, 0.15f, 0.165f, 0.18f};

    std::cout << "simple selection (Andrew's)"<<std::endl;
    std::cout << "alpha \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
    std::array<int,16> simple_selected = {15691, 22088, 18847, 6187, 22013, 3051, 15693, 3025, 
                                            6217, 9355, 18851, 22136, 15719, 6266, 18994, 2356};
    std::vector<Eigen::VectorXf> simple_selected_collision_constraints_;
    simple_selected_collision_constraints_.clear();
    for(auto &ind : simple_selected){
        simple_selected_collision_constraints_.push_back(collision_constraints_[ind]);
    }
    std::array<float, 6> alpha_arr_new = {0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f};
    for(auto &alpha: alpha_arr_new){
        genNormPose<Dim>(poses_noramlized, alpha);
        rtcl::check_constraint(
            bool_constraint,
            poses_noramlized,
            collision_constraints_);

        rtcl::check_constraint(
            bool_selected_constraint,
            poses_noramlized,
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
        genNormPose<Dim>(poses_noramlized, alpha);
        rtcl::check_constraint(
            bool_constraint,
            poses_noramlized,
            collision_constraints_);
        rtcl::check_constraint(
            bool_selected_constraint,
            poses_noramlized,
            selected_collision_constraints_);

        rtcl::count_booleans(counts,
            bool_selected_constraint,
            bool_constraint );

        std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                  << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;
    }


    std::cout << "=================================" << std::endl;
    // CORNER NORMALIZED    
    // example : rss_RO_result_corner_normalized_0.030000.bin
    std::string result_dir_path = CURRENT_DIR "test/testdata/rt-result-sizhe/corner_normalized/";    
    std::cout << "alpha \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
    for(auto &alpha: alpha_arr){
        // read Sizhe's result
        readBin(result_dir_path + "rss_RO_result_corner_normalized_" + std::to_string(alpha)+ ".bin", result);
        for (size_t i = 0; i < result.size(); ++i) {
            bool_ground_truth[i] = result[i]>0;
        }
        // gen samples
        genNormPose<Dim>(poses_noramlized, alpha);
        rtcl::check_constraint(
            bool_constraint,
            poses_noramlized,
            // simple_selected_collision_constraints_);
            selected_collision_constraints_);
            // collision_constraints_);
        rtcl::count_booleans(counts,
            bool_constraint,
            bool_ground_truth );

        std::cout << alpha << "\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                  << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;        
    }

}

TEST_F(CollisionConstraintValidationTest, QuickhullTest){
    
    
    const uint Dim = 8;
    const uint num_samples = pow(3,8)-1; // sampling on [-1,0,1] excluding {0,0..0}

    std::vector<std::array<float, Dim>> poses_noramlized;   
    std::vector<int> result;    
    rtcl::Eigen_::VectorXb bool_constraint;
    rtcl::Eigen_::VectorXb bool_selected_constraint;
    rtcl::Eigen_::VectorXb bool_simple_selected_constraint;

    poses_noramlized.reserve(num_samples);
    result.reserve(num_samples);
    bool_constraint.resize(num_samples);
    bool_selected_constraint.resize(num_samples);
    bool_simple_selected_constraint.resize(num_samples);
           
    std::array<int,4> counts = {0,0,0,0};

    for(uint folder_num(0); folder_num<30; ++folder_num)
    {
        // read cases
        // const uint folder_num = 21;
        read_case(folder_num, joint_config_, obstacle_poses_, obstacle_dimensions_);

        // update collision_constraints_, selected_collision_constraints_
        update_collision_constraints();

        // update bool_simple_selected_constraint
        std::array<int,16> simple_selected;
        read_simple_selection<16>(folder_num, simple_selected);
        std::vector<Eigen::VectorXf> simple_selected_collision_constraints_;
        simple_selected_collision_constraints_.clear();
        for(auto &ind : simple_selected){
            simple_selected_collision_constraints_.push_back(collision_constraints_[ind]);
        }

        std::cout << "folder_num \t FF(True Negative) FT(False Negative) \t TT(True Positive) TF(False Positive)"<<std::endl;
        // quick hull
        float alpha = 0.1f;
        genNormPose<Dim>(poses_noramlized, alpha);
        rtcl::check_constraint(
            bool_constraint,
            poses_noramlized,
            collision_constraints_);
        rtcl::check_constraint(
            bool_selected_constraint,
            poses_noramlized,
            selected_collision_constraints_);
        rtcl::count_booleans(counts,
            bool_selected_constraint,
            bool_constraint );

        std::cout << folder_num << "\t\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;

        rtcl::check_constraint(
            bool_simple_selected_constraint,
            poses_noramlized,
            simple_selected_collision_constraints_);
        rtcl::count_booleans(counts,
            bool_simple_selected_constraint,
            bool_constraint );

        std::cout << folder_num << "\t\t" << counts[0] << "\t\t" << counts[1] << "\t\t\t" 
                << counts[2] << "\t\t" << counts[3] << "\t\t" << std::endl;
        
    }

}


