#include <gtest/gtest.h>

#include "framework/cccb_trajopt/test_interface.hpp"
#include "framework/user_command.hpp"
#include "test/json_utils.h"
// json_list_to_eigen, json_listoflist_to_eigenmat, json_listoflist_to_vecofeigen


void set_obstacles(const nlohmann::json &j_obs, std::vector<OBSTACLE> &obstacles){
    std::cout<<"set_obstacles"<<std::endl;
    obstacles.clear();
    OBSTACLE obstacle;
    for (auto& [key, val] : j_obs.items())
    {
        // std::cout << "key: " << key << ", value:" << val << '\n';
        obstacle.pose = json_list_to_eigen(val["pose"]);
        obstacle.dimension = json_list_to_eigen(val["info"]["data"]);
        obstacles.push_back(obstacle);
    }
}   


void read_case(PLANNING_COMMAND* plancmd, const std::string_view robotname, int casenum){
    std::cout<<" read_case " << std::endl;
    std::stringstream filenamess;
    filenamess << "/home/jelee/my_ws/TrajOpt/test/testdata/8dof-collision/" << robotname << "/case" << casenum <<".json";
    std::string filename;
    filenamess >> filename;
    std::cout<<filename.c_str()<<std::endl;
    std::ifstream fJson(filename);
    std::stringstream buffer;
    buffer << fJson.rdbuf();
    auto problem = nlohmann::json::parse(buffer.str());

    plancmd->max_joint_speed = json_list_to_eigen(problem["max_joint_speed"]);
    plancmd->max_joint_acceleration = json_list_to_eigen(problem["max_joint_acceleration"]);
    plancmd->max_joint_jerk = json_list_to_eigen(problem["max_joint_jerk"]);
    set_obstacles(problem["obstacles"], plancmd->obstacles);

    std::cout<<"set joint_path"<<std::endl;
    json_listoflist_to_vecofeigen(problem["joint_path"], plancmd->joint_path, false);
}

TEST(CCCBTrajOptTest, AddGrippedBox){
    std::string assets_dir = "/home/jelee/my_ws/TrajOpt/rtcl/assets";
    std::string robot_name = "ra830a";
    TestInterface* infc = new TestInterface(robot_name, assets_dir);

    PLANNING_COMMAND* plancmd = new PLANNING_COMMAND();
    SOLUTION* solution = new SOLUTION();
    TRAJ_DATA* traj_data = new TRAJ_DATA();

    // generate_rand_problem(plancmd);    
    read_case(plancmd,robot_name,1);
    plancmd->gripped_box.pose_from_ee << 0.0, 0.0, 0.2032, 1.0, 0.0, 0.0, 0.0;
    plancmd->gripped_box.dimension << 0.4064, 0.4064, 0.4064;
    std::cout << "setting gripped box info"<< std::endl;
    std::cout << plancmd->gripped_box.pose_from_ee.transpose() << std::endl;
    std::cout << plancmd->gripped_box.dimension.transpose() << std::endl;
    infc->updateAlpha(10);
    infc->doPlanning(plancmd);

    SOLUTION* imp_soln = new SOLUTION();

    infc->getPlannedResult(imp_soln);
    double tstep = 0.05;
    infc->getPlannedTrajectory(tstep, traj_data);

    // print results
    // for (int i(0); i<traj_data->tdata.size(); ++i){
    //     std::cout<< traj_data->tdata[i] <<  "," 
    //     << traj_data->qdata[i].transpose() << "," 
    //     << traj_data->dqdata[i].transpose() << std::endl;
    // }
}

// TEST(CCCBTrajOptTest, NoGrippedBox){
//     std::string assets_dir = "/home/jelee/my_ws/TrajOpt/rtcl/assets";
//     std::string robot_name = "ra830a";
//     TestInterface* infc = new TestInterface(robot_name, assets_dir);

//     PLANNING_COMMAND* plancmd = new PLANNING_COMMAND();
//     SOLUTION* solution = new SOLUTION();
//     TRAJ_DATA* traj_data = new TRAJ_DATA();

//     // generate_rand_problem(plancmd);    
//     read_case(plancmd,robot_name,1);
//     infc->updateAlpha(10);
//     infc->doPlanning(plancmd);

//     SOLUTION* imp_soln = new SOLUTION();

//     infc->getPlannedResult(imp_soln);
//     double tstep = 0.05;
//     infc->getPlannedTrajectory(tstep, traj_data);

//     // print results
//     // for (int i(0); i<traj_data->tdata.size(); ++i){
//     //     std::cout<< traj_data->tdata[i] <<  "," 
//     //     << traj_data->qdata[i].transpose() << "," 
//     //     << traj_data->dqdata[i].transpose() << std::endl;
//     // }
// }