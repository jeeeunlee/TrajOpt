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
    alpha_ = 50.;
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

    timer.printElapsedMiliSec("initialize = ");

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
    
    int n_iter(0), max_iter(5);
    timer.printElapsedMiliSec("opt setting = ");
    Eigen::VectorXd x_double;
    while(n_iter++ < max_iter){
     
        // update constraints: Ac*del_cp + ah*delh <= b
        updateConstraints(cp_bar, hbar, Ac, ah, b);
        timer.printElapsedMiliSec("updateConstraints = ");
        // update collision constraints: 
        addColConstraints(cp_bar, hbar, Ac, ah, b);
        timer.printElapsedMiliSec("addColConstraints = ");

        // set constraints
        updateAsparse(Ac,ah,A_sparse);
        timer.printElapsedMiliSec("updateAsparse = ");
        
        // solve problem
        updateQuadCostCoeffs(cp_bar, Q_sparse, q);
        timer.printElapsedMiliSec("updateQuadCostCoeffs = ");
        
        float retf = rossy_utils::qpprogOSQPSparse(Q_sparse, q, A_sparse, b, x);            
        timer.printElapsedMiliSec("qpprogOSQP = ");
        
        // Eigen::Map<Eigen::MatrixXf> xmat(x.data(), dim_, x.size()/dim_);
        // std::cout << "x (dCP) = " << xmat.transpose() << std::endl; 
        // std::cout<<" I'm here 6 " << std::endl;

        // update
        cp_vector = cp_bar + x.segment(0,CPdim);
        h = getMinH(cp_vector, planning_cmd);

        // check terminate conditions
        if(h>hbar){
            std::cout<<"@@ n_iter ["<<n_iter<<"], h="<< h << " => " << N*h << std::endl;
            std::cout<<"[Termination] : h increased from " << hbar <<" to " << h << std::endl;
            break;
        }
        else if(N*(hbar-h)<1e-2){ // (cp_bar-cp_vector).norm()<1e-3
            std::cout<<"@@ n_iter ["<<n_iter<<"], h="<< h << " => " << N*h << std::endl;
            std::cout<<"[Termination] : h decreased from " << hbar <<" to " << h << std::endl;
            break;
        }
        else // (h<hbar)
        {
            std::cout<<"@@ n_iter ["<<n_iter<<"], h="<<h << " => " << N*h <<std::endl;
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
    
    // tuning parameters: min (del_h - alpha/gamma)^2
    // if gamma is too small then Q become ill-conditioned
    float gamma = 1.f; // regulization term
    float alpha = 50.f; // weight term

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
            if(i>dim_){
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
    int dim_vc = Av_.rows();
    int dim_ac = Aa_.rows();
    int dim_jc = Aj_.rows();
    int dim_constr = 2*(dim_vc + dim_ac + dim_jc + 1); // (+,-)
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
        // h constr
        // Ac.block(2*dim_vc+2*dim_ac+2*dim_jc, 0, 2, CPdim) = Eigen::MatrixXf::Zero(2, CPdim);
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

    // h constr: -delh<0, delh<jbar
    ah[2*dim_vc+2*dim_ac+2*dim_jc] = -1.f;
    ah[2*dim_vc+2*dim_ac+2*dim_jc+1] = 1.f;
    b[2*dim_vc+2*dim_ac+2*dim_jc] = 0.f;
    b[2*dim_vc+2*dim_ac+2*dim_jc+1] = hbar;
}

void CCCBTrajOptSolver::addColConstraints(
        const Eigen::VectorXf &cp_bar,
        float hbar,
        Eigen::MatrixXf &Ac,
        Eigen::VectorXf &ah,
        Eigen::VectorXf &b){
    float dist_relaxed = 0.0f; //-0.01;
    Eigen::MatrixXf Actmp, tmp;
    Eigen::VectorXf ahtmp, btmp, btmp1, btmp2;

    int CPdim = cp_bar.size();

    // knot points
    Eigen::VectorXf pVec = Ap_*cp_bar + bp_;
    int num_knot_points = pVec.size()/dim_; // N-1
    std::vector<Eigen::VectorXf> joint_configs(num_knot_points);
    for(int i(0); i<num_knot_points; ++i){
        joint_configs[i] = pVec.segment(i*dim_,dim_) ;
    }

    // compute collision constraints U*Δq < d
    Eigen::MatrixXf U = Eigen::MatrixXf::Zero(0,0);
    Eigen::VectorXf d = Eigen::VectorXf::Zero(0);
    obstacle_manager_->computeCollisionConstraints(joint_configs, U, d);

    // std::cout<<"   - obstacle constraint dimension: " << d.size() << std::endl;
    int NObs = U.rows();

    if(NObs>0){
        // add collision constraints
        // Actmp = U*Ap_;
        ((RtclObstacleManager*)obstacle_manager_)->mapObstacleCoeff(U, Ap_, Actmp);
        ahtmp = Eigen::VectorXf::Zero(NObs,1);
        btmp = d + Eigen::VectorXf::Constant(NObs, dist_relaxed);        
        Ac = rossy_utils::vStack(Ac, Actmp);
        ah = rossy_utils::vStack(ah, ahtmp);
        b = rossy_utils::vStack(b, btmp);

        // add max CPs dist for each
        float rmax = 0.2;
        int Pdim = Ap_.rows();
        tmp = -Ap_;
        Actmp = rossy_utils::vStack(Ap_, tmp);
        ahtmp = Eigen::VectorXf::Zero(2*Pdim, 1);
        btmp = Eigen::VectorXf::Constant(2*Pdim, rmax);    
        Ac = rossy_utils::vStack(Ac, Actmp);
        ah = rossy_utils::vStack(ah, ahtmp);
        b = rossy_utils::vStack(b,btmp);
    }
}

void CCCBTrajOptSolver::updateAsparse(
        const Eigen::MatrixXf& Ac,
        const Eigen::VectorXf& ah,
        Eigen::SparseMatrix<float>& A_sparse){
    // A = [Ac, ah] 
    assert(Ac.rows() == ah.size());
    int dim_constr = 2*(Av_.rows() + Aa_.rows() + Aj_.rows() + 1);    
    int nr = ah.size();
    int nc = Ac.cols() + 1;  

    if(!initialized_){        
        Eigen::SparseMatrix<float>  Ac1_sparse = Ac.topRows(dim_constr).sparseView();
        Ac1_sparse.makeCompressed();
        n_Ac1_nonzero_ = Ac1_sparse.nonZeros();
        Ac_triplets_.resize(n_Ac1_nonzero_);
        int n_triplet(0);
        for (int k = 0; k < Ac1_sparse.outerSize(); ++k) {
            for (Eigen::SparseMatrix<float>::InnerIterator it(Ac1_sparse, k); it; ++it) {
                Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.row(), it.col(), it.value());
            }
        }
    }           

    Eigen::SparseMatrix<float> Ac2_sparse = Ac.bottomRows(nr-dim_constr).sparseView();    
    Eigen::SparseVector<float> ah_sparse = ah.sparseView();    
    Ac2_sparse.makeCompressed();
    
    Ac_triplets_.resize(n_Ac1_nonzero_ + Ac2_sparse.nonZeros() + ah_sparse.nonZeros());
    int n_triplet(n_Ac1_nonzero_);    
    for (int k = 0; k < Ac2_sparse.outerSize(); ++k) {
        for (Eigen::SparseMatrix<float>::InnerIterator it(Ac2_sparse, k); it; ++it) {
            Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.row()+dim_constr, it.col(), it.value());
        }
    }
    for (Eigen::SparseVector<float>::InnerIterator it(ah_sparse); it; ++it) {
        Ac_triplets_[n_triplet++] = Eigen::Triplet<float>(it.index(), nc-1, it.value());
    }
    A_sparse.resize(nr, nc);
    A_sparse.setFromTriplets(Ac_triplets_.begin(), Ac_triplets_.end());
    A_sparse.makeCompressed();
    
    // original
    // Eigen::MatrixXf A = Eigen::MatrixXf::Zero(Ac.rows(), Ac.cols()+1);
    // A.leftCols(Ac.cols()) = Ac;
    // A.col(A.cols()-1) = ah;
    // A_sparse = A.sparseView();
    // A_sparse.makeCompressed();
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

