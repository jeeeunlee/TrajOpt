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
    alpha_ = -1.;
}

bool CCCBTrajOptSolver::solve(PLANNING_COMMAND* planning_cmd){
    // std::cout<<" CCCBTrajOptSolver::solve " <<std::endl;
    // std::cout << "vlimit = " << planning_cmd->max_joint_speed.transpose() << std::endl;
    // std::cout << "alimit = " << planning_cmd->max_joint_acceleration.transpose() << std::endl;
    // std::cout << "jlimit = " << planning_cmd->max_joint_jerk.transpose() << std::endl;
    Clock timer, timer2;
    timer2.start();

    timer.start();
    /* 1. initialize traj */
    // get initial CPs to track given path and h0 that satisfies constraints
    updateCoeffs(planning_cmd, cccb_traj_);    
    Eigen::MatrixXf CPvars0 = 
        cccb_traj_->findBSpline(planning_cmd->joint_path);    

    // CPvars = [cp[0], cp[1],...,] : dim x (N-3) matrix
    // CPvec = [cp[0]; cp[1];...] : dim*(N-3) x 1 vector
    Eigen::VectorXf CPvec0 = rossy_utils::MatrixtoVector(CPvars0);
    float h0 = getMinH(CPvec0, planning_cmd);

    timer.printElapsedMiliSec("initialize = ");

    // 2. optimization: find CPvec, h
    Eigen::VectorXf CPvec;
    float h;
    // CPvec = CPbar + delCP, h = hbar - delh
    // x = [delCP, delh]
    // min c'*x    subject to:   A*x <= b
    // max (delh) = min (-delh) subject to [Ac, ah]*[delCP;delh] <= b
    int CPdim = CPvec0.size(); // dim*(N-3)    
    int N = CPvars0.cols() + 3;
    Eigen::MatrixXf A, Ac; 
    Eigen::VectorXf x, ah, b;
    Eigen::VectorXf c = Eigen::VectorXf::Zero(CPdim+1);
    c(CPdim) = -1.;

    // ADDED for QP: 0.5*x'*Q*x + q'*x 
    Eigen::VectorXf q = Eigen::VectorXf::Zero(0); // CPdim+1
    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(0, 0); // CPdim+1, CPdim+1  

    // initialize norminal vars
    Eigen::VectorXf CPbar = CPvec0;
    float hbar = h0;
    
    int n_iter(0), max_iter(5);
    timer.printElapsedMiliSec("opt setting = ");
    Eigen::VectorXd x_double;
    while(n_iter++ < max_iter){
     
        // update constraints: Ac*delCP + ah*delh <= b
        updateConstraints(CPbar, hbar, Ac, ah, b);
        timer.printElapsedMiliSec("updateConstraints = ");
        addColConstraints(CPbar, hbar, Ac, ah, b);
        timer.printElapsedMiliSec("addColConstraints = ");

        // set constraints
        A = Eigen::MatrixXf::Zero(ah.size(), CPdim+1);
        A << Ac, ah;
        
        // solve problem

        if(alpha_ < 0){
            double ret = rossy_utils::linprog(
                c.cast<double>(), 
                A.cast<double>(), 
                b.cast<double>(), 
                x_double);
            x = x_double.cast<float>();
        }            
        else{
            updateQuadCostCoeffs(CPbar, Q, q);
            q += alpha_*c;
            timer.printElapsedMiliSec("updateQuadCostCoeffs = ");
            float retf = rossy_utils::qpprogOSQP(
                Q, q, A, b, x);            
            timer.printElapsedMiliSec("qpprogOSQP = ");
        }
        // Eigen::Map<Eigen::MatrixXf> xmat(x.data(), dim_, x.size()/dim_);
        // std::cout << "x (dCP) = " << xmat.transpose() << std::endl; 
        // std::cout<<" I'm here 6 " << std::endl;

        // update
        CPvec = CPbar + x.segment(0,CPdim);
        h = getMinH(CPvec, planning_cmd);

        // check terminate conditions
        if(h>hbar){
            std::cout<<"## n_iter ["<<n_iter<<"], h="<< h << "=>" << N*h << std::endl;
            std::cout<<"???? h increased from" << hbar <<" to " << h << std::endl;
            break;
        }
        else if((hbar-h)<1e-3){ // (CPbar-CPvec).norm()<1e-3
            std::cout << "Termination: h decreased from" << hbar <<" to " << h;
            std::cout << " at n_iter = " << n_iter << std::endl;
            break;
        }
        else // (h<hbar)
        {
            std::cout<<"## n_iter ["<<n_iter<<"], h="<<h << "=>" << N*h <<std::endl;
            CPbar = CPvec;
            hbar = h;
        }
        
    }
    CPVec_ = CPbar;
    h_ = hbar;

    std::vector<Eigen::VectorXf> CPvars;
    for(int i(0); i<CPvec.size()/dim_; ++i)
        CPvars.push_back(CPbar.segment(i*dim_, dim_));
    cccb_traj_->setBSpline(pi_,pf_,CPvars);
    cccb_traj_->setTimeDuration(hbar);
    timer2.printElapsedMiliSec("// solve trajopt = ");
    
    // checkSplinePrint();
    // TODO : check soln exist later
    return true;
}

void CCCBTrajOptSolver::updateQuadCostCoeffs(
    const Eigen::VectorXf &CPbar,
    Eigen::MatrixXf &Q,
    Eigen::VectorXf &q)
{
    int dim = dim_;
    int CPdim = CPbar.size();
    int n = (int)(CPdim/dim); // = N-3   

    // update Hessian only if none
    // std::cout<<" dim = " << dim << ", n="<< n  << ", CPdim = " << CPdim << std::endl;
    if(Q.rows() == 0){
        Q = Eigen::MatrixXf::Zero(CPdim+1, CPdim+1);

        Eigen::MatrixXf Q1d = Eigen::MatrixXf::Zero(n,n);
        Q1d.block(0,0,n,n) += 4.*Eigen::MatrixXf::Identity(n,n);
        Q1d.block(1,0,n-1,n-1) += -2.*Eigen::MatrixXf::Identity(n-1,n-1);
        Q1d.block(0,1,n-1,n-1) += -2.*Eigen::MatrixXf::Identity(n-1,n-1);
        // std::cout <<"Q1d = "<<std::endl;
        // std::cout << Q1d << std::endl;
        Eigen::MatrixXf Qx = Q1d; // n x n
        if(dim>1){
            Eigen::MatrixXf repmat = Eigen::MatrixXf::Identity(dim,dim);
            Qx = rossy_utils::kroneckerProduct(Q1d, repmat); // dn x dn
        }
        // std::cout<<"Qx = "<<std::endl;
        // std::cout<< Qx << std::endl;        
        Q.block(0,0,CPdim,CPdim) = Qx;
        Q(CPdim,CPdim) = 0.001; // just for regulation
    }    
    // std::cout<<"Q = "<<std::endl;
    // std::cout<< Q << std::endl;

    Eigen::VectorXf Cpi0,Cpi,Cpi1;
    q = Eigen::VectorXf::Zero(CPdim+1);
    for(int i(0); i<n; ++i)
    {
        Cpi = CPbar.segment(i*dim, dim);

        if(i==0){ Cpi0 = pi_;}
        else {Cpi0 = CPbar.segment((i-1)*dim, dim);}
        
        if(i==n-1){ Cpi1 = pf_;}
        else {Cpi1 = CPbar.segment((i+1)*dim, dim);}

        q.segment(i*dim, dim) = 2.*(- Cpi0 + 2.*Cpi - Cpi1);
    }
} 

// void CCCBTrajOptSolver::updateQuadCostCoeffs(
//     const Eigen::VectorXf &CPbar,
//     Eigen::MatrixXf &Q,
//     Eigen::VectorXf &q)
// {
//     int dim = dim_;
//     int CPdim = CPbar.size();
//     int n = (int)(CPdim/dim); // = N-3   

//     // update Hessian only if none
//     if(Q.rows() == 0){
//         std::cout <<" dim = " << dim << ", n="<< n  
//                   <<", CPdim = " << CPdim << std::endl;

//         Q = Eigen::MatrixXf::Zero(CPdim+1, CPdim+1);   
//         Q.block(0,0,CPdim,CPdim) = Ap_.transpose()*Ap_;
//         Q(CPdim,CPdim) = 0.001; // just for regulation
//     }    
//     q = Eigen::VectorXf::Zero(CPdim+1);

// } 



void CCCBTrajOptSolver::updateConstraints(const Eigen::VectorXf &CPbar,
                                        float hbar, 
                                        Eigen::MatrixXf &Ac,
                                        Eigen::VectorXf &ah,
                                        Eigen::VectorXf &b){
    Eigen::MatrixXf Actmp, tmp;
    Eigen::VectorXf ahtmp, btmp, btmp1, btmp2;
    int CPdim = CPbar.size();

    // vel constr
    tmp = -Av_;
    Ac = rossy_utils::vStack(Av_, tmp);
    ah = rossy_utils::vStack(VCrep_, VCrep_);
    btmp1 = hbar*VCrep_ - Av_*CPbar - bv_;
    btmp2 = hbar*VCrep_ + Av_*CPbar + bv_;
    b = rossy_utils::vStack(btmp1, btmp2);

    // acc constr
    tmp = -Aa_;
    Actmp = rossy_utils::vStack(Aa_, tmp);
    ahtmp = 2.*hbar*rossy_utils::vStack(ACrep_, ACrep_);
    btmp1 = hbar*hbar*ACrep_ - Aa_*CPbar - ba_;
    btmp2 = hbar*hbar*ACrep_ + Aa_*CPbar + ba_;
    btmp = rossy_utils::vStack(btmp1, btmp2);
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

    // jerk constr
    tmp = -Aj_;
    Actmp = rossy_utils::vStack(Aj_, tmp);
    ahtmp = 3.*hbar*hbar*rossy_utils::vStack(JCrep_, JCrep_);
    btmp1 = hbar*hbar*hbar*JCrep_ - Aj_*CPbar - bj_;
    btmp2 = hbar*hbar*hbar*JCrep_ + Aj_*CPbar + bj_;
    btmp = rossy_utils::vStack(btmp1, btmp2);
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

    // horizon constr
    Actmp = Eigen::MatrixXf::Zero(2, CPdim);
    ahtmp = Eigen::VectorXf::Zero(2);
    ahtmp << -1,1;
    btmp = Eigen::VectorXf::Zero(2);
    btmp << 0, hbar;
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

}

void CCCBTrajOptSolver::addColConstraints(const Eigen::VectorXf &CPbar,
                                        float hbar,
                                        Eigen::MatrixXf &Ac,
                                        Eigen::VectorXf &ah,
                                        Eigen::VectorXf &b){
    float dist_relaxed = 0.0f; //-0.01;
    Eigen::MatrixXf Actmp, tmp;
    Eigen::VectorXf ahtmp, btmp, btmp1, btmp2;

    int CPdim = CPbar.size();

    // knot points
    std::vector<Eigen::VectorXf> joint_configs;
    Eigen::VectorXf pVec = Ap_*CPbar + bp_;
    for(int i(0); i<pVec.size()/dim_; ++i){
        // std::cout << " knot q =" << pVec.segment(i*dim_,dim_).transpose() << std::endl;
        joint_configs.push_back(pVec.segment(i*dim_,dim_)) ;
    }      

    // compute collision constraints U*Δq < d
    Eigen::MatrixXf U = Eigen::MatrixXf::Zero(0,0);
    Eigen::VectorXf d = Eigen::VectorXf::Zero(0);
    obstacle_manager_->updateObstacleCoeff(joint_configs, U, d);

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

void CCCBTrajOptSolver::getKnotValues(SOLUTION * soln){
    soln->h = h_;
    Eigen::VectorXf tmp = Ap_*CPVec_ + bp_;
    // Eigen::MatrixXf p = rossy_utils::VectortoMatrix(tmp,2);
    soln->path.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->path.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Av_*CPVec_ + bv_)/h_;
    // Eigen::MatrixXf v = rossy_utils::VectortoMatrix(tmp,2);
    soln->velocity.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->velocity.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aa_*CPVec_ + ba_)/h_/h_;
    // Eigen::MatrixXf a = rossy_utils::VectortoMatrix(tmp,2);
    soln->acceleration.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->acceleration.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aj_*CPVec_ + bj_)/h_/h_/h_;
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


float CCCBTrajOptSolver::getMinH(const Eigen::VectorXf &CPvec,
                                PLANNING_COMMAND* planning_cmd){
    // vel*h = Av*CP + bv : dim*(N-2)
    Eigen::VectorXf velh = Av_ * CPvec + bv_;
    
    // acc*h*h = Aa*CP + ba : dim*(N-1)
    Eigen::VectorXf acch2 = Aa_ * CPvec + ba_;

    // jerk*h*h*h = Aj*CP + bj : dim*N
    Eigen::VectorXf jerkh3 = Aj_ * CPvec + bj_;

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

