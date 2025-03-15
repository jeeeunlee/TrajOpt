#include "framework/cccb_trajopt/cccb_trajopt_solver.hpp"
#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "framework/cccb_trajopt/rtcl_obstacle_manager.hpp"
#include "framework/cccb_trajopt/no_obstacle_manager.hpp"
#include "rossy_utils/solvers/lp_solver.hpp"
#include "rossy_utils/solvers/qp_solver.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "cccb_trajopt_solver.hpp"

// for benchmark
#include "rossy_utils/general/clock.hpp"

CCCBTrajOptSolver::CCCBTrajOptSolver(CCCBTrajManager* _cccb_traj, 
                                    ObstacleManager* _obstacle_manager)
    : cccb_traj_(_cccb_traj), obstacle_manager_(_obstacle_manager){
}

bool CCCBTrajOptSolver::solve(PLANNING_COMMAND* planning_cmd){
    // std::cout<<" CCCBTrajOptSolver::solve " <<std::endl;
    Clock timer, timer2;
    initialized_ = false;

    /* 1. initialize traj */
    // get initial CPs to track given path and h0 that satisfies constraints
    updateCoeffs(planning_cmd, cccb_traj_); 
    Eigen::MatrixXf cp_variables_0 = cccb_traj_->findBSpline(planning_cmd->joint_path); //  [cp[0], cp[1],...,] : dim x (N-3) matrix
    Eigen::VectorXf cp_0 = rossy_utils::MatrixtoVector(cp_variables_0); // [cp[0]; cp[1];...] : dim*(N-3) x 1 vector
    float h0 = getMinH(cp_0, planning_cmd);    

    // 2. optimization: find cp_vector, h
    // cp_vector = cp_bar + del_cp, h = hbar - delh
    // x = [del_cp, delh]
    // min c'*x    subject to:   A*x <= b
    // max (delh) = min (-delh) subject to [Ac, ah]*[del_cp;delh] <= b
    int CPdim = cp_0.size(); // dim*(N-3)
    int N = cp_variables_0.cols() + 3;
    Eigen::MatrixXf Ac;
    Eigen::SparseMatrix<float> A_sparse;
    Eigen::VectorXf x, ah, b;

    // ADDED for QP: 0.5*x'*Q*x + q'*x 
    Eigen::VectorXf q = Eigen::VectorXf::Zero(0); // CPdim+1
    // Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(0, 0); // CPdim+1, CPdim+1  
    Eigen::SparseMatrix<float> Q_sparse; 
    // initialize norminal vars
    Eigen::VectorXf cp_vector, cp_bar = cp_0;
    float h, hbar = h0;
    
    int n_iter(0), max_iter(10);
    timer.printElapsedMiliSec("initialize = ");
    std::cout<<"@@ n_iter ["<<n_iter<<"], h="<< hbar << " => " << N*hbar << std::endl;
    rossy_utils::OSQPSolver solver;
    Eigen::MatrixXf Ac_clsn;
    Eigen::VectorXf ah_col, b_clsn;
    while(n_iter++ < max_iter){
     
        // update constraints: Ac*del_cp + ah*delh <= b
        updateQuadCostCoeffs(cp_bar, Q_sparse, q);
        updateConstraints(cp_bar, hbar, Ac, ah, b);
        timer.printElapsedMiliSec("updateConstraints = ");
        // update collision constraints: 
        updateColConstraints(cp_bar, hbar, Ac_clsn, b_clsn);       
        // set final constraints for OSQP
        updateAsparseb(Ac, ah, Ac_clsn, b_clsn, A_sparse, b);
        timer.printElapsedMiliSec("updateColConstraints(ray-traced) = ");
        // timer.printElapsedMiliSec("updateAsparseb = ");
        
        // solve problem
        float retf = solver.qpprogOSQPSparse(Q_sparse, q, A_sparse, b, x);
        timer.printElapsedMiliSec("qpprogOSQP = ");

        // update
        cp_vector = cp_bar + x.segment(0,CPdim);
        h = getMinH(cp_vector, planning_cmd);

        // check terminate conditions
        float tt_reduced = N*(hbar-h);
        float h_change = hbar-h;
        float h_diff_relative = abs(hbar-h)/hbar;
        float cp_diff_relative = (cp_bar-cp_vector).norm()/cp_bar.norm();
        if( cp_diff_relative < 5e-3 || h_diff_relative < 5e-3  ){ // || h_diff < 5e-3
            std::cout<<"@@ n_iter ["<<n_iter<<"], h="<< h << " => " << N*h << std::endl;
            // std::cout<<"   retf = " << retf << ", h_diff(rel,abs) = " << h_diff_relative << ", " << h_change <<
            //         ", cp_diff(rel,abs) = " << cp_diff_relative << ", " << cp_change << std::endl;
            std::cout<< "[Termination]" << std::endl;
            cp_bar = cp_vector;
            hbar = h;
            break;
        }
        // else if(h_change < 0){
        //     std::cout<<"@@ n_iter ["<<n_iter<<"], h="<< h << " => " << N*h << std::endl;
        //     std::cout<< "[Termination]" << std::endl;
        //     break;
        // }
        else // (h<hbar)
        {
            std::cout<<"@@ n_iter ["<<n_iter<<"], h="<<h << " => " << N*h <<std::endl;
            // std::cout<<"   retf = " << retf << ", h_diff(rel,abs) = " << h_diff_relative << ", " << h_change <<
            //         ", cp_diff(rel,abs) = " << cp_diff_relative << ", " << cp_change << std::endl;
            cp_bar = cp_vector;
            hbar = h;
        }
        initialized_ = true;        
    }
    cp_vector_ = cp_bar;
    h_ = hbar;

    std::vector<Eigen::VectorXf> CPvars;
    for(int i(0); i<cp_vector.size()/dim_; ++i)
        CPvars.push_back(cp_bar.segment(i*dim_, dim_));
    cccb_traj_->setBSpline(pi_,pf_,CPvars);
    cccb_traj_->setTimeDuration(hbar);
    timer2.printElapsedMiliSec("// solve trajopt = ");
    
    // checkSplinePrint();
    // TODO : check soln exist later
    return true;
}

void CCCBTrajOptSolver::updateQuadCostCoeffs(
    const Eigen::VectorXf &cp_bar,
    Eigen::SparseMatrix<float> &Q_sparse,
    Eigen::VectorXf &q)
{
    int CPdim = cp_bar.size();
    int n = (int)(CPdim/dim_); // = N-3  
    
    // tuning parameters: min 0.5*gamma*(del_h - alpha/gamma)^2 ~ -alpha*delh
    // if gamma is too small then Q become ill-conditioned
    float gamma = 5e-4f; // regulization term
    float alpha = 7.5f; // weight term

    // update Hessian only if none
    if( Q_sparse.rows() !=  CPdim+1 ){
        // Qx = [2*I_d, -I_d, 0, ... 0; 
        //      -I_d, 2*I_d, -I_d, ...0; 
        //      0, ... -I_d, 2*I_d, -I_d; 
        //      0, 0, ... -I_d, 2*I_d] : (CPdim) x (CPdim)
        // Q = [Qx, 0; 0, gamma] : (CPdim+1) x (CPdim+1) 
        std::vector<Eigen::Triplet<float>> triplets;
        triplets.resize((n + 2*(n-1))*dim_ + 1); // Each block is dim_ x dim_
        int idx(0);
        for(int i=0; i<CPdim; ++i){
            // col-major order
            if(i>=dim_){
                triplets[idx++] = Eigen::Triplet<float>(i-dim_,i,-1.f);
            }            
            triplets[idx++] = Eigen::Triplet<float>(i,i,2.f);
            if(i<CPdim-dim_){
                triplets[idx++] = Eigen::Triplet<float>(i+dim_,i,-1.f);
            }
        }
        triplets[(n + 2*(n-1))*dim_] = Eigen::Triplet<float>(CPdim,CPdim,gamma);
        Q_sparse.resize(CPdim+1, CPdim+1);
        Q_sparse.setFromTriplets(triplets.begin(), triplets.end());
        Q_sparse.makeCompressed();
    }
    // std::cout<< Q_sparse << std::endl;    
    q = Eigen::VectorXf::Zero(CPdim+1);
    q.segment(0, CPdim) = 2.*cp_bar;
    q.segment(0, dim_) -= pi_;
    q.segment(dim_, CPdim-dim_) -= cp_bar.segment(0, CPdim-dim_);
    q.segment(0, CPdim-dim_) -= cp_bar.segment(dim_, CPdim-dim_);       
    q.segment(CPdim-dim_, dim_) -= pf_;
    q[CPdim] = - alpha;

} 

void CCCBTrajOptSolver::updateConstraints(
        const Eigen::VectorXf &cp_bar,
        float hbar, 
        Eigen::MatrixXf &Ac,
        Eigen::VectorXf &ah,
        Eigen::VectorXf &b){   

    int CPdim = cp_bar.size(); // (N-3)*dim
    int dim_pc = Ap_.rows();
    int dim_vc = Av_.rows();
    int dim_ac = Aa_.rows();
    int dim_jc = Aj_.rows();
    int dim_constr = 2*(dim_vc + dim_ac + dim_jc + dim_pc + 1); // (+,-)
    // update Ac only if it's first
    if(!initialized_){
        Ac = Eigen::MatrixXf::Zero(dim_constr, CPdim);        
        // vel constr
        Ac.block(0, 0, dim_vc, CPdim) = Av_;
        Ac.block(dim_vc, 0, dim_vc, CPdim) = - Av_;
        // acc constr
        Ac.block(2*dim_vc, 0, dim_ac, CPdim) = Aa_;      
        Ac.block(2*dim_vc+dim_ac, 0, dim_ac, CPdim) = - Aa_;
        // jerk constr
        Ac.block(2*dim_vc+2*dim_ac, 0, dim_jc, CPdim) = Aj_;      
        Ac.block(2*dim_vc+2*dim_ac+dim_jc, 0, dim_jc, CPdim) = - Aj_;
        // position constr: -rmax < Ap_*dC + 0*dh < rmax
        Ac.block(2*dim_vc+2*dim_ac+2*dim_jc, 0, dim_pc, CPdim) = Ap_;
        Ac.block(2*dim_vc+2*dim_ac+2*dim_jc+dim_pc, 0, dim_pc, CPdim) = -Ap_;
        // h constr
        // Ac.block(2*dim_vc+2*dim_ac+2*dim_jc+2*dim_pc, 0, 2, CPdim) = Eigen::MatrixXf::Zero(2, CPdim);
    }
    else{
        Ac = Ac.topRows(dim_constr);
    }
    ah = Eigen::VectorXf::Zero(dim_constr);
    b = Eigen::VectorXf::Zero(dim_constr);

     // vel constr 
    ah.segment(0, dim_vc) = VCrep_;
    ah.segment(dim_vc, dim_vc) = VCrep_;
    b.segment(0, dim_vc) = hbar*VCrep_ - Av_*cp_bar - bv_;
    b.segment(dim_vc, dim_vc) = hbar*VCrep_ + Av_*cp_bar + bv_;

    // acc constr 
    ah.segment(2*dim_vc, dim_ac) = 2.*hbar*ACrep_;
    ah.segment(2*dim_vc+dim_ac, dim_ac) = 2.*hbar*ACrep_;
    b.segment(2*dim_vc, dim_ac) = hbar*hbar*ACrep_ - Aa_*cp_bar - ba_;
    b.segment(2*dim_vc+dim_ac, dim_ac) = hbar*hbar*ACrep_ + Aa_*cp_bar + ba_;

    // jerk constr
    ah.segment(2*dim_vc+2*dim_ac, dim_jc) = 3.*hbar*hbar*JCrep_;
    ah.segment(2*dim_vc+2*dim_ac+dim_jc, dim_jc)= 3.*hbar*hbar*JCrep_;
    b.segment(2*dim_vc+2*dim_ac, dim_jc) = hbar*hbar*hbar*JCrep_ - Aj_*cp_bar - bj_;
    b.segment(2*dim_vc+2*dim_ac+dim_jc, dim_jc)= hbar*hbar*hbar*JCrep_ + Aj_*cp_bar + bj_;

    // position constr: -rmax < Ap_*dC + 0*dh < rmax
    float rmax = 0.15f;
    b.segment(2*dim_vc+2*dim_ac+2*dim_jc, dim_pc) = Eigen::VectorXf::Constant(dim_pc, rmax);        
    b.segment(2*dim_vc+2*dim_ac+2*dim_jc+dim_pc, dim_pc) = Eigen::VectorXf::Constant(dim_pc, rmax);

    // h constr: -delh<0, delh<jbar
    ah[2*dim_vc+2*dim_ac+2*dim_jc+2*dim_pc] = -1.f;
    ah[2*dim_vc+2*dim_ac+2*dim_jc+2*dim_pc+1] = 1.f;
    b[2*dim_vc+2*dim_ac+2*dim_jc+2*dim_pc] = 0.f;
    b[2*dim_vc+2*dim_ac+2*dim_jc+2*dim_pc+1] = hbar;
}

void CCCBTrajOptSolver::updateColConstraints(
        const Eigen::VectorXf &cp_bar,
        float hbar,
        Eigen::MatrixXf &Ac_clsn,
        Eigen::VectorXf &b_clsn){

    int CPdim = cp_bar.size();

    // knot points
    Eigen::VectorXf pVec = Ap_*cp_bar + bp_;
    int num_knot_points = pVec.size()/dim_; // N-1
    std::vector<Eigen::VectorXf> joint_configs(num_knot_points);
    for(int i(0); i<num_knot_points; ++i){
        joint_configs[i] = pVec.segment(i * dim_, dim_);
    }

    // compute collision constraints U*Δq < d
    Eigen::MatrixXf U = Eigen::MatrixXf::Zero(0,0);
    b_clsn = Eigen::VectorXf::Zero(0);
    obstacle_manager_->computeCollisionConstraints(joint_configs, U, b_clsn);

    if(U.rows()>0){
        // add collision constraints: U*Ap_*dC + 0*dh < d
        ((RtclObstacleManager*)obstacle_manager_)->mapObstacleCoeff(
            U, Ap_, Ac_clsn); // Actmp = U*Ap_;
        // [Nc*(N-1) x (N-3)*dim] = [Nc*(N-1) x (N-1)*dim] * [(N-1)*dim x (N-3)*dim]
    }
}

void CCCBTrajOptSolver::updateAsparseb(
        const Eigen::MatrixXf& Ac, // dynamic constraints
        const Eigen::VectorXf& ah,
        const Eigen::MatrixXf& Ac_clsn, // collision constraints
        const Eigen::VectorXf& b_clsn,
        Eigen::SparseMatrix<float>& A_sparse,
        Eigen::VectorXf& b){
    assert(Ac.rows() == ah.size());
    assert(Ac_clsn.rows() == b_clsn.size());
    assert(Ac_clsn.cols() == Ac.cols());
    // update b
    b.conservativeResize(b.size() + b_clsn.size());
    b.tail(b_clsn.size()) = b_clsn;

    // A = [Ac, ah] 
    int n_dyn_constr = ah.size();    
    int n_total_constraints = n_dyn_constr + Ac_clsn.rows();
    int dim_cp = Ac.cols();

    if(!initialized_){      
        // Ac is fixed for N and dim
        Eigen::SparseMatrix<float>  Ac1_sparse = Ac.sparseView();
        Ac1_sparse.makeCompressed();
        n_Ac_dyn_nonzero_ = Ac1_sparse.nonZeros();
        Ac_triplets_.resize(n_Ac_dyn_nonzero_);
        int n_triplet(0);
        for (int k = 0; k < Ac1_sparse.outerSize(); ++k) {
            for (Eigen::SparseMatrix<float>::InnerIterator it(Ac1_sparse, k); it; ++it) {
                Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.row(), it.col(), it.value());
            }
        }
    }

    int n_triplet = n_Ac_dyn_nonzero_;
    Eigen::SparseVector<float> ah_sparse = ah.sparseView();
    
    // original code
    // Eigen::SparseMatrix<float> Ac_clsn_sparse = Ac_clsn.sparseView(); 
    // Ac_clsn_sparse.makeCompressed();
    // Ac_triplets_.resize(n_Ac_dyn_nonzero_ + ah_sparse.nonZeros() + Ac_clsn_sparse.nonZeros());
    // for (Eigen::SparseVector<float>::InnerIterator it(ah_sparse); it; ++it) {
    //     Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.index(), dim_cp, it.value());
    // }
    // for (int k = 0; k < Ac_clsn_sparse.outerSize(); ++k) {
    //     for (Eigen::SparseMatrix<float>::InnerIterator it(Ac_clsn_sparse, k); it; ++it) {
    //         Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.row()+n_constr, it.col(), it.value());
    //     }
    // }

    // Assume the fixed Nc: num of collision constraints for each knot point
    // Ac_clsn = U*Ap_ : Nc*(N-1) x (N-3)*dim
    //         = [U1*1/6, 0, ,,                     ]
    //           [U2*2/3, U1*1/6, 0, ,,             ]
    //           [U3*1/6, U2*2/3, U1*1/6, 0, ,,     ]
    //           [0, U4*1/6, U3*2/3, U2*1/6, 0, ,,  ]
    //           [0, 0, U5*1/6, U4*2/3, U3*1/6, 0,  ]
    // Ui : Nc x dim   
    uint N = dim_cp/dim_ + 3; // dim_cp = (N-3)*dim_
    uint Nc = Ac_clsn.rows()/(N-1);
    uint Ac_clsn_nonzero = 3*dim_cp*Nc;
    

    Ac_triplets_.resize(n_Ac_dyn_nonzero_ + ah_sparse.nonZeros() + Ac_clsn_nonzero);
    for (Eigen::SparseVector<float>::InnerIterator it(ah_sparse); it; ++it) {
        Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.index(), dim_cp, it.value());
    }

    size_t ir_base, ic_base;
    for(size_t indU(0); indU<N-3; ++indU){
        for(size_t ind3(0); ind3<3; ++ind3){
            ir_base = (indU+ind3) * Nc;
            ic_base = indU * dim_;
            for(size_t ir(ir_base); ir<ir_base+Nc; ++ir){
                for(size_t ic(ic_base); ic<ic_base+dim_; ++ic){
                    Ac_triplets_[n_triplet++] 
                        = Eigen::Triplet<float>(ir + n_dyn_constr, ic, Ac_clsn(ir,ic));
                }
            }
        }
    }
    

    A_sparse.resize(n_total_constraints, dim_cp+1);
    A_sparse.setFromTriplets(Ac_triplets_.begin(), Ac_triplets_.end());
    A_sparse.makeCompressed();
}


void CCCBTrajOptSolver::getKnotValues(SOLUTION * soln){
    soln->h = h_;
    Eigen::VectorXf tmp = Ap_*cp_vector_ + bp_;
    // Eigen::MatrixXf p = rossy_utils::VectortoMatrix(tmp,2);
    soln->path.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->path.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Av_*cp_vector_ + bv_)/h_;
    // Eigen::MatrixXf v = rossy_utils::VectortoMatrix(tmp,2);
    soln->velocity.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->velocity.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aa_*cp_vector_ + ba_)/h_/h_;
    // Eigen::MatrixXf a = rossy_utils::VectortoMatrix(tmp,2);
    soln->acceleration.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->acceleration.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aj_*cp_vector_ + bj_)/h_/h_/h_;
    // Eigen::MatrixXf j = rossy_utils::VectortoMatrix(tmp,2);
    soln->jerk.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->jerk.push_back( tmp.segment(i*dim_,dim_) ); 
}




void CCCBTrajOptSolver::updateCoeffs(PLANNING_COMMAND* planning_cmd, 
                                CCCBTrajManager* cccb_traj){
    
    N_ = planning_cmd->joint_path.size()+1;
    dim_ = planning_cmd->joint_path[0].size();    
    pi_ = planning_cmd->joint_path[0];
    pf_ = planning_cmd->joint_path[N_-2];

    // std::cout << "updateCoeffs : ";
    // std::cout << "N_ = " << N_ << ", ";
    // std::cout << "dim_ = " << dim_ << std::endl;
    // std::cout << "pi_ = " << pi_.transpose() << std::endl;
    // std::cout << "pf_ = " << pf_.transpose() << std::endl;
                            
    Ap_ = cccb_traj->computeAp(N_,dim_);
    Av_ = cccb_traj->computeAv2(N_,dim_);
    Aa_ = cccb_traj->computeAa(N_,dim_);
    Aj_ = cccb_traj->computeAj(N_,dim_);
    bp_ = cccb_traj->computebp(N_,dim_,pi_,pf_);
    bv_ = cccb_traj->computebv2(N_,dim_,pi_,pf_);
    ba_ = cccb_traj->computeba(N_,dim_,pi_,pf_);
    bj_ = cccb_traj->computebj(N_,dim_,pi_,pf_);

    VCrep_ = (planning_cmd->max_joint_speed).replicate(N_-2,1);
    ACrep_ = (planning_cmd->max_joint_acceleration).replicate(N_-1,1);
    JCrep_ = (planning_cmd->max_joint_jerk).replicate(N_,1);
}


float CCCBTrajOptSolver::getMinH(const Eigen::VectorXf &cp_vector,
                                PLANNING_COMMAND* planning_cmd){
    // vel*h = Av*CP + bv : dim*(N-2)
    Eigen::VectorXf velh = Av_ * cp_vector + bv_;
    
    // acc*h*h = Aa*CP + ba : dim*(N-1)
    Eigen::VectorXf acch2 = Aa_ * cp_vector + ba_;

    // jerk*h*h*h = Aj*CP + bj : dim*N
    Eigen::VectorXf jerkh3 = Aj_ * cp_vector + bj_;

    // get optimal h that satisfies VC,AC,JC: dim*1
    Eigen::VectorXf hvec = rossy_utils::elementWiseDivisionExt(
        velh, planning_cmd->max_joint_speed);
    float hv1 = hvec.cwiseAbs().maxCoeff();
    hvec = rossy_utils::elementWiseDivisionExt(
        acch2, planning_cmd->max_joint_acceleration);
    float ha2 = hvec.cwiseAbs().maxCoeff(); 
    hvec = rossy_utils::elementWiseDivisionExt(
        jerkh3, planning_cmd->max_joint_jerk);
    float hj3 = hvec.cwiseAbs().maxCoeff();     

    float h = std::max(hv1, std::sqrt(ha2));
    h = std::max(h, std::pow(hj3,1.f/3.f));

    // std::cout<<" CCCBTrajOptSolver::getMinH:  h = " << h << ", hv=" << 
    // hv1 << ", ha=" << std::sqrt(ha2) << ", hj=" << std::pow(hj3,1.f/3.f) << std::endl;
    return h;
}

