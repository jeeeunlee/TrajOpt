#include <gtest/gtest.h>

// #include "rossy_utils/math/cccb_spline.hpp"
// #include "rossy_utils/io/io_utilities.hpp"
// #include "framework/cccb_trajopt/cccb_traj_planner.hpp"

#include <fstream>
#include <iostream>
#include "rossy_utils/io/json.hpp"
#include "framework/cccb_trajopt/test_interface.hpp"
#include "framework/user_command.hpp"

void generate_rand_problem(PLANNING_COMMAND* plancmd){
    plancmd->joint_path.clear();

    int N = 30;
    int d = 2;
    Eigen::VectorXd pi = {-1,-1};
    Eigen::VectorXd pf = {1, 1};

    plancmd->max_joint_speed = Eigen::VectorXd::Constant(d, 0.6);
    plancmd->max_joint_acceleration = Eigen::VectorXd::Constant(d, 0.3);
    plancmd->max_joint_jerk = Eigen::VectorXd::Constant(d, 0.45);

    // random path generator
    int n = N-2; // pi = p[0],..., p[n]=pf
    plancmd->joint_path.push_back(pi);

}

Eigen::VectorXd json_list_to_eigen(const nlohmann::json &j){
    Eigen::VectorXd vector(j.size());
    int element_index=0;    
    for (const auto& element : j) {
        vector(element_index++) = (double)element;
    }
    return vector;
}

Eigen::MatrixXd json_listoflist_to_eigenmat(const nlohmann::json &j){
    Eigen::MatrixXd mat(j[0].size(),j.size());
    size_t element_index=0;
    for (const auto& element : j) {
        mat.col(element_index++) = json_list_to_eigen(element);
    }
    return mat;
}

void json_listoflist_to_vecofeigen(const nlohmann::json &j, 
                    std::vector<Eigen::VectorXd> &vector){
    Eigen::MatrixXd mat =json_listoflist_to_eigenmat(j);    
    vector.clear();
    for(int i(0); i<mat.rows(); ++i)
        vector.push_back(mat.row(i));
}


void read_case(PLANNING_COMMAND* plancmd, SOLUTION* solution){
    std::ifstream fJson("/home/jelee/my_ws/TrajOpt/test/testdata/2d-non-collision/case2.json");

    std::stringstream buffer;
    buffer << fJson.rdbuf();
    auto json = nlohmann::json::parse(buffer.str());

    auto problem = json["problem"];

    const size_t dim = problem["dim"];    
    Eigen::VectorXd pi = json_list_to_eigen(problem["pi"]);
    Eigen::VectorXd pf = json_list_to_eigen(problem["pf"]);

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

// 4: rs020, 5: panda
TEST(CCCBTrajOptTest, CheckNoColCase){
    TestInterface* infc = new TestInterface();

    PLANNING_COMMAND* plancmd = new PLANNING_COMMAND();
    SOLUTION* solution = new SOLUTION();
    TRAJ_DATA* traj_data = new TRAJ_DATA();

    // generate_rand_problem(plancmd);
    read_case(plancmd, solution);

    infc->doPlanning(plancmd);

    WPT_DATA* knot_path = new WPT_DATA();
    WPT_DATA* knot_vel = new WPT_DATA();
    WPT_DATA* knot_acc = new WPT_DATA();
    WPT_DATA* knot_jerk = new WPT_DATA();

    infc->getPlannedResult(knot_path, knot_vel, knot_acc, knot_jerk);

    double EPS = 1e-8;
    for(int i(0); i<solution->path.size() ; ++i)    
        EXPECT_NEAR((solution->path[i]-knot_path->data[i]).norm(), 0., EPS);
    for(int i(0); i<solution->velocity.size() ; ++i)    
        EXPECT_NEAR((solution->velocity[i]-knot_vel->data[i]).norm(), 0., EPS);
    for(int i(0); i<solution->acceleration.size() ; ++i)    
        EXPECT_NEAR((solution->acceleration[i]-knot_acc->data[i]).norm(), 0., EPS);
    for(int i(0); i<solution->jerk.size() ; ++i)    
        EXPECT_NEAR((solution->jerk[i]-knot_jerk->data[i]).norm(), 0., EPS);

    infc->getPlannedTrajectory(0.05, traj_data);

    // print results
    // for (int i(0); i<traj_data->tdata.size(); ++i){
    //     std::cout<< traj_data->tdata[i] <<  "," 
    //     << traj_data->qdata[i].transpose() << "," 
    //     << traj_data->dqdata[i].transpose() << std::endl;
    // }
}
