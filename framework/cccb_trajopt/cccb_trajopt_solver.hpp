#pragma once

#include "framework/user_command.hpp"
#include <Eigen/Sparse>

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

        void updateColConstraints(const Eigen::VectorXf &Xbar,
                                float hbar,
                                Eigen::MatrixXf &Ac_clsn,
                                Eigen::VectorXf &b_clsn);

        void updateQuadCostCoeffs(const Eigen::VectorXf &cp_bar,
                                Eigen::SparseMatrix<float> &Q_sparse,
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
        bool initialized_;
        Eigen::MatrixXf cp_vector_;
        float h_;

        // coeff
        void updateCoeffs(PLANNING_COMMAND* planning_cmd, 
                        CCCBTrajManager* cccb_traj);

        void updateAsparseb(
            const Eigen::MatrixXf& Ac,
            const Eigen::VectorXf& ah,
            const Eigen::MatrixXf& Ac_clsn,
            const Eigen::VectorXf& b_clsn,
            Eigen::SparseMatrix<float>& A_sparse,
            Eigen::VectorXf& b);
        std::vector<Eigen::Triplet<float>> Ac_triplets_;
        uint n_Ac_dyn_nonzero_;


        Eigen::MatrixXf Ap_; // ((N-1)*dim) x ((N-3)*dim)
        Eigen::MatrixXf Av_; // ((N-2)*dim) x ((N-3)*dim)
        Eigen::MatrixXf Aa_; // ((N-1)*dim) x ((N-3)*dim)
        Eigen::MatrixXf Aj_; // (N*dim) x ((N-3)*dim)
        Eigen::VectorXf bp_; // (N-1)*dim 
        Eigen::VectorXf bv_; // (N-2)*dim
        Eigen::VectorXf ba_; // (N-1)*dim
        Eigen::VectorXf bj_; // (N)*dim 

        Eigen::VectorXf pi_;
        Eigen::VectorXf pf_;

        int N_;
        int dim_;

        Eigen::VectorXf VCrep_;
        Eigen::VectorXf ACrep_;
        Eigen::VectorXf JCrep_;




};