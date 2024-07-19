#pragma once

#include "framework/user_command.hpp"

// functions on cccb splines
class ObstacleManager;
class CCCBTrajManager;
class Clock;


class CCCBTrajOptSolver{
    public:
        CCCBTrajOptSolver();
        ~CCCBTrajOptSolver(){ }
        bool solve(PLANNING_COMMAND* planning_cmd, 
                ObstacleManager* obstacles, 
                CCCBTrajManager* cccb_traj_);
        double getMinH(const Eigen::VectorXd &CPvec,
                        PLANNING_COMMAND* planning_cmd, 
                        CCCBTrajManager* cccb_traj);
        void updateConstraints(const Eigen::VectorXd &Xbar,
                                double hbar, 
                                Eigen::MatrixXd &Ac,
                                Eigen::VectorXd &ah,
                                Eigen::VectorXd &b);
        void addColConstraints(const Eigen::VectorXd &Xbar,
                                double hbar,
                                ObstacleManager* obstacles,
                                Eigen::MatrixXd &Ac,
                                Eigen::VectorXd &ah,
                                Eigen::VectorXd &b);

        // for check
        void getKnotValues(WPT_DATA * knot_path, 
                            WPT_DATA * knot_vel, 
                            WPT_DATA * knot_acc, 
                            WPT_DATA * knot_jerk);

    public:
        Clock* clock_;

    private:
        // solution
        Eigen::MatrixXd CPVec_;
        double h_;

        // coeff
        void updateCoeffs(PLANNING_COMMAND* planning_cmd, 
                        CCCBTrajManager* cccb_traj);

        Eigen::MatrixXd Ap_;
        Eigen::MatrixXd Av_;
        Eigen::MatrixXd Aa_;
        Eigen::MatrixXd Aj_;
        Eigen::VectorXd bp_;
        Eigen::VectorXd bv_;
        Eigen::VectorXd ba_;
        Eigen::VectorXd bj_;

        Eigen::VectorXd pi_;
        Eigen::VectorXd pf_;

        int N_;
        int dim_;

        Eigen::VectorXd VCrep_;
        Eigen::VectorXd ACrep_;
        Eigen::VectorXd JCrep_;




};