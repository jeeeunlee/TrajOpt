#include <gtest/gtest.h>

#include "rtcl/wrapper/rtcl_interface.h"
#include "framework/user_command.hpp"

void read_info(const uint folder_num, 
                std::vector<OBSTACLE>& obstacles,
                Eigen::VectorXf& joint_config){
    std::cout <<" read info " << std::endl;
    std::ostringstream data_path;
    data_path << "/home/dexterity/ambyld/TrajOpt/experiment_results";
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

    obstacles.clear();
    OBSTACLE obstacle;
    myfile.open(data_path.str() +"boxes.txt");
    while (std::getline(myfile, line)) {
        std::stringstream ss(line);
        std::vector<float> values;
        float value;
        // Parse the line into individual float values
        while (ss >> value) {
            values.push_back(value);
        }
        obstacle.pose.head(3) = Eigen::Vector3f(values[0], values[1], values[2]);
        obstacle.dimension << values[3], values[4], values[5];
        obstacle.dimension *= 2.f; // dim = halfdim * 2
        obstacles.push_back(obstacle);
    }
    myfile.close();
}   




TEST(RTCLTest, AddGrippedBox){
    std::string assets_dir = "/home/dexterity/ambyld/TrajOpt/rtcl/assets";
    std::string robot_name = "ra830a";
    rtcl::RtclInterface* rtcl_interface =  new rtcl::RtclInterface(robot_name, assets_dir);

    // set obstacles
    const uint folder_num = 3;
    std::vector<OBSTACLE> obstacles;
    Eigen::VectorXf joint_config;
    read_info(folder_num, obstacles, joint_config);
    std::vector<Eigen::VectorXf> pose_list;
    std::vector<Eigen::Vector3f> dim_list;
    for(auto &obs: obstacles){
        const Eigen::VectorXf pose{obs.pose};
        const Eigen::Vector3f dim{obs.dimension};
        // std::cout << "pose = "<< pose.transpose() << std::endl;
        // std::cout << "dim = "<< dim.transpose() << std::endl;
        pose_list.push_back(pose);
        dim_list.push_back(dim);            
    }
    rtcl_interface->clearBoxObstacles();
    rtcl_interface->setBoxObstacles(pose_list, dim_list);

    // set gripped box
    Eigen::VectorXf pose_from_ee {{0.0f, 0.0f, 0.2032f, 1.0f, 0.0f, 0.0f, 0.0f}};
    Eigen::Vector3f dimension {{0.4064f, 0.4064f, 0.4064f}};
    rtcl_interface->clearGrippedBox();
    rtcl_interface->setGrippedBox(pose_from_ee, 
                                dimension);

    // compute collision constraints for single joint configuration
    bool robot_collision_free = rtcl_interface->checkJointConfigCollisionDistance(joint_config);
    
    rtcl::CollisionCheckerData debug_data;
    rtcl_interface->loadDebugData(debug_data);

}

