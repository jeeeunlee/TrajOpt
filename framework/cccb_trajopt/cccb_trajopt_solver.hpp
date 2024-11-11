#pragma once

#include "framework/user_command.hpp"

// functions on cccb splines
class ObstacleManager;
class CCCBTrajManager;
class Clock;


class CCCBTrajOptSolver{
    public:
        CCCBTrajOptSolver(CCCBTrajManager* _cccb_traj, 
                        ObstacleManager* _obstacle_manager);
        ~CCCBTrajOptSolver(){ }
        bool solve(PLANNING_COMMAND* planning_cmd);
        float getMinH(const Eigen::VectorXf &CPvec,
                        PLANNING_COMMAND* planning_cmd);
        void updateConstraints(const Eigen::VectorXf &Xbar,
                                float hbar, 
                                Eigen::MatrixXf &Ac,
                                Eigen::VectorXf &ah,
                                Eigen::VectorXf &b);
        void addColConstraints(const Eigen::VectorXf &Xbar,
                                float hbar,
                                Eigen::MatrixXf &Ac,
                                Eigen::VectorXf &ah,
                                Eigen::VectorXf &b);

        void updateQuadCostCoeffs(const Eigen::VectorXf &CPbar,
                                Eigen::MatrixXf &Q,
                                Eigen::VectorXf &q);

        // for check
        void getKnotValues(SOLUTION * soln);

    public:
        float alpha_;

    private:
        // instances
        CCCBTrajManager* cccb_traj_; 
        ObstacleManager* obstacle_manager_;

        // solution
        Eigen::MatrixXf CPVec_;
        float h_;

        // coeff
        void updateCoeffs(PLANNING_COMMAND* planning_cmd, 
                        CCCBTrajManager* cccb_traj);

        Eigen::MatrixXf Ap_;
        Eigen::MatrixXf Av_;
        Eigen::MatrixXf Aa_;
        Eigen::MatrixXf Aj_;
        Eigen::VectorXf bp_;
        Eigen::VectorXf bv_;
        Eigen::VectorXf ba_;
        Eigen::VectorXf bj_;

        Eigen::VectorXf pi_;
        Eigen::VectorXf pf_;

        int N_;
        int dim_;

        Eigen::VectorXf VCrep_;
        Eigen::VectorXf ACrep_;
        Eigen::VectorXf JCrep_;




};