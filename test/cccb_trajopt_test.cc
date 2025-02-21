#include <gtest/gtest.h>

#include "framework/cccb_trajopt/test_interface_nocol.hpp"
#include "framework/user_command.hpp"
#include "test/json_utils.h"
#include "Configuration.h"

void generate_rand_problem(PLANNING_COMMAND* plancmd){
    plancmd->joint_path.clear();

    int N = 30;
    int d = 2;
    Eigen::VectorXf pi = {-1,-1};
    Eigen::VectorXf pf = {1, 1};

    plancmd->max_joint_speed = Eigen::VectorXf::Constant(d, 0.6);
    plancmd->max_joint_acceleration = Eigen::VectorXf::Constant(d, 0.3);
    plancmd->max_joint_jerk = Eigen::VectorXf::Constant(d, 0.45);

    // random path generator
    int n = N-2; // pi = p[0],..., p[n]=pf
    plancmd->joint_path.push_back(pi);

}

void read_case(PLANNING_COMMAND* plancmd, SOLUTION* solution, int casenum){   
    std::stringstream filenamess;
    filenamess << CURRENT_DIR "test/testdata/2d-non-collision/case" << casenum <<".json";
    std::string filename;
    filenamess >> filename;
    std::ifstream fJson(filename);

    std::stringstream buffer;
    buffer << fJson.rdbuf();
    auto json = nlohmann::json::parse(buffer.str());

    auto problem = json["problem"];

    const size_t dim = problem["dim"];    
    Eigen::VectorXf pi = json_list_to_eigen(problem["pi"]);
    Eigen::VectorXf pf = json_list_to_eigen(problem["pf"]);

    plancmd->max_joint_speed = json_list_to_eigen(problem["VC"]);
    plancmd->max_joint_acceleration = json_list_to_eigen(problem["AC"]);
    plancmd->max_joint_jerk = json_list_to_eigen(problem["JC"]);
    json_listoflist_to_vecofeigen(problem["path"], plancmd->joint_path);
    
    solution->h = json["solution"]["h"];
    json_listoflist_to_vecofeigen(json["solution"]["path"], solution->path);
    json_listoflist_to_vecofeigen(json["solution"]["vel"], solution->velocity);
    json_listoflist_to_vecofeigen(json["solution"]["acc"], solution->acceleration);
    json_listoflist_to_vecofeigen(json["solution"]["jerk"], solution->jerk);
}


TEST(CCCBTrajOptTest, CheckNoColCase){
    std::string panda_urdf = CURRENT_DIR "simulator/configs/urdf_files/franka_panda.urdf";
    NoColTestInterface* infc = new NoColTestInterface(panda_urdf);

    PLANNING_COMMAND* plancmd = new PLANNING_COMMAND();
    SOLUTION* solution = new SOLUTION();
    TRAJ_DATA* traj_data = new TRAJ_DATA();

    // generate_rand_problem(plancmd);
    read_case(plancmd, solution, 2);

    infc->doPlanning(plancmd);

    SOLUTION* imp_soln = new SOLUTION();

    infc->getPlannedResult(imp_soln);

    float EPS = 1e-8;
    for(int i(0); i<solution->path.size() ; ++i)    
        EXPECT_NEAR((solution->path[i]-imp_soln->path[i]).norm(), 0., EPS);
    for(int i(0); i<solution->velocity.size() ; ++i)    
        EXPECT_NEAR((solution->velocity[i]-imp_soln->velocity[i]).norm(), 0., EPS);
    for(int i(0); i<solution->acceleration.size() ; ++i)    
        EXPECT_NEAR((solution->acceleration[i]-imp_soln->acceleration[i]).norm(), 0., EPS);
    for(int i(0); i<solution->jerk.size() ; ++i)    
        EXPECT_NEAR((solution->jerk[i]-imp_soln->jerk[i]).norm(), 0., EPS);

    infc->getPlannedTrajectory(0.05, traj_data);

    // print results
    // for (int i(0); i<traj_data->tdata.size(); ++i){
    //     std::cout<< traj_data->tdata[i] <<  "," 
    //     << traj_data->qdata[i].transpose() << "," 
    //     << traj_data->dqdata[i].transpose() << std::endl;
    // }
}

TEST(CCCBTrajOptTest, CheckNoColCaseQP){
    std::string panda_urdf = CURRENT_DIR "simulator/configs/urdf_files/franka_panda.urdf";
    NoColTestInterface* infc = new NoColTestInterface(panda_urdf);

    PLANNING_COMMAND* plancmd = new PLANNING_COMMAND();
    SOLUTION* solution = new SOLUTION();
    TRAJ_DATA* traj_data = new TRAJ_DATA();

    // generate_rand_problem(plancmd);
    read_case(plancmd, solution, 3);
    infc->updateAlpha(50);
    infc->doPlanning(plancmd);

    SOLUTION* imp_soln = new SOLUTION();

    infc->getPlannedResult(imp_soln);

    float EPS = 1e-8;
    for(int i(0); i<solution->path.size() ; ++i)    
        EXPECT_NEAR((solution->path[i]-imp_soln->path[i]).norm(), 0., EPS);
    // for(int i(0); i<solution->velocity.size() ; ++i)    
    //     EXPECT_NEAR((solution->velocity[i]-imp_soln->velocity[i]).norm(), 0., EPS);
    // for(int i(0); i<solution->acceleration.size() ; ++i)    
    //     EXPECT_NEAR((solution->acceleration[i]-imp_soln->acceleration[i]).norm(), 0., EPS);
    // for(int i(0); i<solution->jerk.size() ; ++i)    
    //     EXPECT_NEAR((solution->jerk[i]-imp_soln->jerk[i]).norm(), 0., EPS);

    infc->getPlannedTrajectory(0.05, traj_data);

    // print results
    // for (int i(0); i<traj_data->tdata.size(); ++i){
    //     std::cout<< traj_data->tdata[i] <<  "," 
    //     << traj_data->qdata[i].transpose() << "," 
    //     << traj_data->dqdata[i].transpose() << std::endl;
    // }
    std::cout<<" path (soln), (impl)"<<std::endl;
    for(int i(0); i<solution->path.size() ; ++i) {
        std::cout << std::right << std::setprecision(4) << std::setw(4) << i << ": ";
        std::cout << std::right << std::setw(10) << solution->path[i].transpose() << " / ";
        std::cout << std::right << std::setw(10) << imp_soln->path[i].transpose() << std::endl;
    }
}
