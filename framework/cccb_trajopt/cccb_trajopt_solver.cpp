#include "framework/cccb_trajopt/cccb_trajopt_solver.hpp"
#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "framework/cccb_trajopt/obstacle_manager.hpp"
#include "rossy_utils/solvers/lp_solver.hpp"
#include "rossy_utils/solvers/qp_solver.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "cccb_trajopt_solver.hpp"

CCCBTrajOptSolver::CCCBTrajOptSolver()
{
    alpha_ = -1.;
}

bool CCCBTrajOptSolver::solve(PLANNING_COMMAND* planning_cmd, 
        ObstacleManager* obstacles, 
        CCCBTrajManager* cccb_traj){
    /* 0. initialize obstacles */
    obstacles->setObstacles(planning_cmd->obstacles);

    /* 1. initialize traj */
    // get initial CPs to track given path and h0 that satisfies constraints
    updateCoeffs(planning_cmd, cccb_traj);
    Eigen::MatrixXd CPvars0 = 
        cccb_traj->findBSpline(planning_cmd->joint_path);    

    // CPvars = [cp[0], cp[1],...,] : dim x (N-3) matrix
    // CPvec = [cp[0]; cp[1];...] : dim*(N-3) x 1 vector
    Eigen::VectorXd CPvec0 = rossy_utils::MatrixtoVector(CPvars0);
    double h0 = getMinH(CPvec0, planning_cmd);

    // 2. optimization: find CPvec, h
    Eigen::VectorXd CPvec;
    double h;
    // CPvec = CPbar + delCP, h = hbar - delh
    // x = [delCP, delh]
    // min c'*x    subject to:   A*x <= b
    // max (delh) = min (-delh) subject to [Ac, ah]*[delCP;delh] <= b
    int CPdim = CPvec0.size(); // dim*(N-3)    
    Eigen::MatrixXd A, Ac; 
    Eigen::VectorXd x, ah, b;
    Eigen::VectorXd c = Eigen::VectorXd::Zero(CPdim+1);
    c(CPdim) = -1.;

    // ADDED for QP: 0.5*x'*Q*x + q'*x 
    int Nf = planning_cmd->joint_path.size();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(0); // CPdim+1
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(0, 0); // CPdim+1, CPdim+1  

    // initialize norminal vars
    Eigen::VectorXd CPbar = CPvec0;
    double hbar = h0;
    
    int n_iter(0), max_iter(1);
    while(n_iter++ < max_iter){
     
        // update constraints: Ac*delCP + ah*delh <= b
        updateConstraints(CPbar, hbar, Ac, ah, b);
        // addColConstraints(CPbar, hbar, obstacles, Ac, ah, b);

        // set constraints
        A = Eigen::MatrixXd::Zero(ah.size(), CPdim+1);
        A << Ac, ah;

        // solve problem
        if(alpha_ < 0)
            double ret = rossy_utils::linprog(c, A, b, x);
        else{
            updateQuadCostCoeffs(CPbar, Q, q);
            q += alpha_*c;
            double ret = rossy_utils::qpprogHiGHS(Q, q, A, b, x);
            // double ret = rossy_utils::qpprog(Q, q, -A, -b, x);
        }
        // std::cout<<" I'm here 6 " << std::endl;

        // update
        CPvec = CPbar + x.segment(0,CPdim);
        h = getMinH(CPvec, planning_cmd);

        // check terminate conditions
        if(h>hbar){
            std::cout<<"???? n_iter ["<<n_iter<<"], h="<<h<<std::endl;
            std::cout<<"???? h increased from" << hbar <<" to " << h << std::endl;
            CPbar = CPvec;
            hbar = h;
            break;
        }
        else if((hbar-h)<1e-3){ // (CPbar-CPvec).norm()<1e-3
            std::cout << "Termination: h decreased from" << hbar <<" to " << h;
            std::cout << " at n_iter = " << n_iter << std::endl;
            break;
        }
        else // (h<hbar)
        {
            std::cout<<"n_iter ["<<n_iter<<"], h="<<h<<std::endl;
            CPbar = CPvec;
            hbar = h;
        }
        
    }
    CPVec_ = CPbar;
    h_ = hbar;

    std::vector<Eigen::VectorXd> CPvars;
    for(int i(0); i<CPvec.size()/dim_; ++i)
        CPvars.push_back(CPbar.segment(i*dim_, dim_));
    cccb_traj->setBSpline(pi_,pf_,CPvars);
    cccb_traj->setTimeDuration(hbar);
    
    // checkSplinePrint();
    // TODO : check soln exist later
    return true;
}

void CCCBTrajOptSolver::updateQuadCostCoeffs(
    const Eigen::VectorXd &CPbar,
    Eigen::MatrixXd &Q,
    Eigen::VectorXd &q)
{
    std::cout<<" I'm here : updateQuadCostCoeffs " << std::endl;
    int dim = dim_;
    int CPdim = CPbar.size();
    int n = (int)(CPdim/dim); // = N-3   

    // update Hessian only if none
    std::cout<<" dim = " << dim << ", n="<< n  << ", CPdim = " << CPdim << std::endl;
    if(Q.rows() == 0){
        Q = Eigen::MatrixXd::Zero(CPdim+1, CPdim+1);

        Eigen::MatrixXd Q1d = Eigen::MatrixXd::Zero(n,n);
        Q1d.block(0,0,n,n) += 4.*Eigen::MatrixXd::Identity(n,n);
        Q1d.block(1,0,n-1,n-1) += -2.*Eigen::MatrixXd::Identity(n-1,n-1);
        Q1d.block(0,1,n-1,n-1) += -2.*Eigen::MatrixXd::Identity(n-1,n-1);
        // std::cout <<"Q1d = "<<std::endl;
        // std::cout << Q1d << std::endl;
        Eigen::MatrixXd Qx = Q1d; // n x n
        if(dim>1){
            Eigen::MatrixXd repmat = Eigen::MatrixXd::Identity(dim,dim);
            Qx = rossy_utils::kroneckerProduct(Q1d, repmat); // dn x dn
        }
        // std::cout<<"Qx = "<<std::endl;
        // std::cout<< Qx << std::endl;        
        Q.block(0,0,CPdim,CPdim) = Qx;
        // Q(CPdim,CPdim) = 0.001; // just for regulation
    }    
    // std::cout<<"Q = "<<std::endl;
    // std::cout<< Q << std::endl;

    Eigen::VectorXd Cpi0,Cpi,Cpi1;
    q = Eigen::VectorXd::Zero(CPdim+1);
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



void CCCBTrajOptSolver::updateConstraints(const Eigen::VectorXd &CPbar,
                                        double hbar, 
                                        Eigen::MatrixXd &Ac,
                                        Eigen::VectorXd &ah,
                                        Eigen::VectorXd &b){
    Eigen::MatrixXd Actmp;
    Eigen::VectorXd ahtmp, btmp, btmp1, btmp2;
    int CPdim = CPbar.size();

    // vel constr
    Ac = rossy_utils::vStack(Av_, -Av_);
    ah = rossy_utils::vStack(VCrep_, VCrep_);
    btmp1 = hbar*VCrep_ - Av_*CPbar - bv_;
    btmp2 = hbar*VCrep_ + Av_*CPbar + bv_;
    b = rossy_utils::vStack(btmp1, btmp2);

    // acc constr
    Actmp = rossy_utils::vStack(Aa_, -Aa_);
    ahtmp = 2.*hbar*rossy_utils::vStack(ACrep_, ACrep_);
    btmp1 = hbar*hbar*ACrep_ - Aa_*CPbar - ba_;
    btmp2 = hbar*hbar*ACrep_ + Aa_*CPbar + ba_;
    btmp = rossy_utils::vStack(btmp1, btmp2);
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

    // jerk constr
    Actmp = rossy_utils::vStack(Aj_, -Aj_);
    ahtmp = 3.*hbar*hbar*rossy_utils::vStack(JCrep_, JCrep_);
    btmp1 = hbar*hbar*hbar*JCrep_ - Aj_*CPbar - bj_;
    btmp2 = hbar*hbar*hbar*JCrep_ + Aj_*CPbar + bj_;
    btmp = rossy_utils::vStack(btmp1, btmp2);
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

    // horizon constr
    Actmp = Eigen::MatrixXd::Zero(2, CPdim);
    ahtmp = Eigen::VectorXd::Zero(2);
    ahtmp << -1,1;
    btmp = Eigen::VectorXd::Zero(2);
    btmp << 0, hbar;
    Ac = rossy_utils::vStack(Ac, Actmp);
    ah = rossy_utils::vStack(ah, ahtmp);
    b = rossy_utils::vStack(b,btmp);

}

void CCCBTrajOptSolver::addColConstraints(const Eigen::VectorXd &Xbar,
                                        double hbar,
                                        ObstacleManager* obstacles,
                                        Eigen::MatrixXd &Ac,
                                        Eigen::VectorXd &ah,
                                        Eigen::VectorXd &b){
    double dist_relaxed = -0.01;
    Eigen::MatrixXd Actmp;
    Eigen::VectorXd ahtmp, btmp, btmp1, btmp2;

    // compute collision constraints
    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(0,0);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(0);
    obstacles->updateObstacleCoeff(U,d);
    int NObs = U.rows();

    if(NObs>0){
        // add collision constraints
        Actmp = U*Ap_;
        ahtmp = Eigen::VectorXd::Zero(NObs,1);
        btmp = d + Eigen::VectorXd::Constant(NObs, dist_relaxed);        
        Ac = rossy_utils::vStack(Ac, Actmp);
        ah = rossy_utils::vStack(ah, ahtmp);
        b = rossy_utils::vStack(b,btmp);

        // add max CPs dist for each
        double rmax = 0.1;
        int Pdim = Ap_.rows();
        Actmp = rossy_utils::vStack(Ap_, -Ap_);
        ahtmp = Eigen::VectorXd::Zero(2*Pdim, 1);
        btmp = Eigen::VectorXd::Constant(2*Pdim, rmax);    
        Ac = rossy_utils::vStack(Ac, Actmp);
        ah = rossy_utils::vStack(ah, ahtmp);
        b = rossy_utils::vStack(b,btmp);
    }
}

void CCCBTrajOptSolver::getKnotValues(SOLUTION * soln){
    soln->h = h_;
    Eigen::VectorXd tmp = Ap_*CPVec_ + bp_;
    // Eigen::MatrixXd p = rossy_utils::VectortoMatrix(tmp,2);
    soln->path.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->path.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Av_*CPVec_ + bv_)/h_;
    // Eigen::MatrixXd v = rossy_utils::VectortoMatrix(tmp,2);
    soln->velocity.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->velocity.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aa_*CPVec_ + ba_)/h_/h_;
    // Eigen::MatrixXd a = rossy_utils::VectortoMatrix(tmp,2);
    soln->acceleration.clear();
    for(int i(0); i<tmp.size()/dim_; ++i)
        soln->acceleration.push_back( tmp.segment(i*dim_,dim_) );

    tmp = (Aj_*CPVec_ + bj_)/h_/h_/h_;
    // Eigen::MatrixXd j = rossy_utils::VectortoMatrix(tmp,2);
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

    std::cout << "updateCoeffs : ";
    std::cout << "N_ = " << N_ << ", ";
    std::cout << "dim_ = " << dim_ << ", ";
    std::cout << "pi_ = " << pi_.transpose() << ", ";
    std::cout << "pf_ = " << pf_.transpose() << std::endl;
                            
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


double CCCBTrajOptSolver::getMinH(const Eigen::VectorXd &CPvec,
                                PLANNING_COMMAND* planning_cmd){
    // vel*h = Av*CP + bv : dim*(N-2)
    Eigen::VectorXd velh = Av_ * CPvec + bv_;
    
    // acc*h*h = Aa*CP + ba : dim*(N-1)
    Eigen::VectorXd acch2 = Aa_ * CPvec + ba_;

    // jerk*h*h*h = Aj*CP + bj : dim*N
    Eigen::VectorXd jerkh3 = Aj_ * CPvec + bj_;

    // get optimal h that satisfies VC,AC,JC: dim*1
    Eigen::VectorXd hvec = rossy_utils::elementWiseDivisionExt(
        velh, planning_cmd->max_joint_speed);
    double hv1 = hvec.cwiseAbs().maxCoeff();
    hvec = rossy_utils::elementWiseDivisionExt(
        acch2, planning_cmd->max_joint_acceleration);
    double ha2 = hvec.cwiseAbs().maxCoeff(); 
    hvec = rossy_utils::elementWiseDivisionExt(
        jerkh3, planning_cmd->max_joint_jerk);
    double hj3 = hvec.cwiseAbs().maxCoeff();     

    double h = std::max(hv1, std::sqrt(ha2));
    h = std::max(h, std::pow(hj3,1./3.));

    std::cout<<" I'm here 3: h = " << h << ", hv=" << 
    hv1 << ", ha=" << std::sqrt(ha2) << ", hj=" << std::pow(hj3,1./3.) << std::endl;
    return h;
}

