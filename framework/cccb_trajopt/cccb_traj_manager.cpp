// functions on cccb splines

#include "framework/cccb_trajopt/cccb_traj_manager.hpp"
#include "rossy_utils/math/cccb_spline.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/io/io_utilities.hpp"
#include "cccb_traj_manager.hpp"

#define MIN_NUM_WPTS 5

CCCBTrajManager::CCCBTrajManager(){
    rossy_utils::pretty_constructor(1, "TrajectoryManager");
    // spline_t2q_.initialize()
    spline_t2q_ = new CCCBSplineVec();
}

CCCBTrajManager::~CCCBTrajManager()
{ delete spline_t2q_;}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd){
    q_cmd = spline_t2q_->evaluate(t);
}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd, 
    Eigen::VectorXd & qdot_cmd) {
    q_cmd = spline_t2q_->evaluate(t);
    qdot_cmd = spline_t2q_->evaluateFirstDerivative(t);    
}

void CCCBTrajManager::getCommand(double t, 
    Eigen::VectorXd & q_cmd, 
    Eigen::VectorXd & qdot_cmd, 
    Eigen::VectorXd & qddot_cmd) {
    q_cmd = spline_t2q_->evaluate(t);
    qdot_cmd = spline_t2q_->evaluateFirstDerivative(t); 
    qddot_cmd = spline_t2q_->evaluateSecondDerivative(t);
}


double CCCBTrajManager::getMotionPeriod()
{
    return spline_t2q_->getMotionPeriod();
}






// Based on Clamped Cardinal Cubic B-spline properties:

void CCCBTrajManager::setBSpline(const Eigen::VectorXd &pi, 
            const Eigen::VectorXd &pf,
            const std::vector<Eigen::VectorXd> &cp_in){
    // set spline_t2q_
    spline_t2q_->setControlPoints(pi,pf,cp_in);
}
void CCCBTrajManager::setTimeDuration(const double & h_in){
    spline_t2q_->setTimeDuration(h_in);
}

Eigen::MatrixXd CCCBTrajManager::findBSpline(
    const std::vector< Eigen::VectorXd > & joint_path){
    // find a set of CPs(control points) of b-spline that tracks
    // the given path: [p[0],p[1],...,p[n]]: dim x (n+1)
    // CPs = [cp[-3,-2,-1], cp[0],...,cp[N-4], cp[N-3,N-2,N-1]]
    assert(joint_path.size()>MIN_NUM_WPTS-1);
    int N = joint_path.size()+1;
    int dim = joint_path[0].size();
    
    Eigen::VectorXd pi = joint_path[0];
    Eigen::VectorXd pf = joint_path[N-2];
    Eigen::MatrixXd pathmat = rossy_utils::vector2EigenMatrix(joint_path);

    // Ap1d*cp1d + bp1d = p1d
    Eigen::MatrixXd Ap1d;
    Eigen::VectorXd bp1d, p1d, cp1d;

    Ap1d = computeAp(N,1);
    Ap1d = Ap1d.block(1,0,N-3,N-3).eval(); // remove pi,pf
   
    Eigen::MatrixXd CPs = Eigen::MatrixXd::Zero(dim,N-3);
    for(int d(0); d<dim; ++d){
        // find CPs for d-th joint
        bp1d = computebp(N,1,pi.segment(d,1),pf.segment(d,1));
        bp1d = bp1d.segment(1,N-3).eval(); // remove pi,pf
        p1d = pathmat.row(d);
        p1d = p1d.segment(1,N-3).eval(); // remove pi,pf
        cp1d = Ap1d.fullPivHouseholderQr().solve(p1d-bp1d);
        // set CPs for d-th joint 
        CPs.row(d) = cp1d;
    }
    return CPs;
}

Eigen::MatrixXd CCCBTrajManager::computeAp(int N, int dim){
    // Ap1d = (N-1)x(N-3) matrix
    Eigen::MatrixXd Ap1d = Eigen::MatrixXd::Zero(N-1,N-3);
    Ap1d.block(0,0,N-3,N-3) += 1./6.*Eigen::MatrixXd::Identity(N-3,N-3);
    Ap1d.block(1,0,N-3,N-3) += 2./3.*Eigen::MatrixXd::Identity(N-3,N-3);
    Ap1d.block(2,0,N-3,N-3) += 1./6.*Eigen::MatrixXd::Identity(N-3,N-3);
    // stack dim: Ap = ((N-1)*dim) x ((N-3)*dim)
    return stack1dMatDim(dim, Ap1d);   
}

Eigen::VectorXd CCCBTrajManager::computebp(int N, int dim, 
        const Eigen::VectorXd &pi, const Eigen::VectorXd &pf){
    assert(pi.size()==dim && pf.size()==dim);
    // bp = (N-1)*dim x 1
    Eigen::VectorXd bp = Eigen::VectorXd::Zero((N-1)*dim);
    bp.segment(0,dim) = 5./6.*pi;
    bp.segment(dim,dim) = 1./6.*pi;
    bp.segment((N-3)*dim,dim) = 1./6.*pf;
    bp.segment((N-2)*dim,dim) = 5./6.*pf;
    return bp;
}

Eigen::MatrixXd CCCBTrajManager::computeAv2(int N, int dim){    
    // Av1d = (N-1)x(N-3) matrix
    Eigen::MatrixXd Av1d = Eigen::MatrixXd::Zero(N-2,N-3);
    Av1d.block(0,0,N-3,N-3) += Eigen::MatrixXd::Identity(N-3,N-3);
    Av1d.block(1,0,N-3,N-3) += -Eigen::MatrixXd::Identity(N-3,N-3);    
    // stack dim: Av = ((N-1)*dim) x ((N-3)*dim)
    return stack1dMatDim(dim, Av1d);
}

Eigen::VectorXd CCCBTrajManager::computebv2(int N, int dim,
        const Eigen::VectorXd &pi, const Eigen::VectorXd &pf){
    assert(pi.size()==dim && pf.size()==dim);
    // bv = (N-1)*dim x 1
    Eigen::VectorXd bv = Eigen::VectorXd::Zero((N-2)*dim);
    bv.segment(0,dim) = -pi;
    bv.segment((N-3)*dim,dim) = pf;
    return bv;
}

Eigen::MatrixXd CCCBTrajManager::computeAv(int N, int dim){    
    // Av1d = (N-1)x(N-3) matrix
    Eigen::MatrixXd Av1d = Eigen::MatrixXd::Zero(N-1,N-3);
    Av1d.block(0,0,N-3,N-3) += 0.5*Eigen::MatrixXd::Identity(N-3,N-3);
    Av1d.block(2,0,N-3,N-3) += -0.5*Eigen::MatrixXd::Identity(N-3,N-3);    
    // stack dim: Av = ((N-1)*dim) x ((N-3)*dim)
    return stack1dMatDim(dim, Av1d);
}

Eigen::VectorXd CCCBTrajManager::computebv(int N, int dim,
        const Eigen::VectorXd &pi, const Eigen::VectorXd &pf){
    assert(pi.size()==dim && pf.size()==dim);
    // bv = (N-1)*dim x 1
    Eigen::VectorXd bv = Eigen::VectorXd::Zero((N-1)*dim);
    bv.segment(0,dim) = -0.5*pi;
    bv.segment(dim,dim) = -0.5*pi;
    bv.segment((N-3)*dim,dim) = 0.5*pf;
    bv.segment((N-2)*dim,dim) = 0.5*pf;
    return bv;
}

Eigen::MatrixXd CCCBTrajManager::computeAa(int N, int dim){
    // Aa1d = (N-1)x(N-3) matrix
    Eigen::MatrixXd Aa1d = Eigen::MatrixXd::Zero(N-1,N-3);
    Aa1d.block(0,0,N-3,N-3) += Eigen::MatrixXd::Identity(N-3,N-3);
    Aa1d.block(1,0,N-3,N-3) += -2.0*Eigen::MatrixXd::Identity(N-3,N-3);
    Aa1d.block(2,0,N-3,N-3) += Eigen::MatrixXd::Identity(N-3,N-3);
    // stack dim: Aa = ((N-1)*dim) x ((N-3)*dim)
    return stack1dMatDim(dim, Aa1d);
}

Eigen::VectorXd CCCBTrajManager::computeba(int N, int dim, 
        const Eigen::VectorXd &pi, const Eigen::VectorXd &pf){
    assert(pi.size()==dim && pf.size()==dim);
    // ba = (N-1)*dim x 1
    Eigen::VectorXd ba = Eigen::VectorXd::Zero((N-1)*dim);
    ba.segment(0,dim) = -pi;
    ba.segment(dim,dim) = pi;
    ba.segment((N-3)*dim,dim) = pf;
    ba.segment((N-2)*dim,dim) = -pf;
    return ba;
}

Eigen::MatrixXd CCCBTrajManager::computeAj(int N, int dim){
    // Aj1d = (N)x(N-3) matrix
    Eigen::MatrixXd Aj1d = Eigen::MatrixXd::Zero(N,N-3);
    Aj1d.block(0,0,N-3,N-3) += Eigen::MatrixXd::Identity(N-3,N-3);
    Aj1d.block(1,0,N-3,N-3) += -3.*Eigen::MatrixXd::Identity(N-3,N-3);
    Aj1d.block(2,0,N-3,N-3) += 3.*Eigen::MatrixXd::Identity(N-3,N-3);
    Aj1d.block(3,0,N-3,N-3) += -Eigen::MatrixXd::Identity(N-3,N-3);
    // stack dim: Aj = (N*dim) x ((N-3)*dim)
    return stack1dMatDim(dim, Aj1d);
}

Eigen::VectorXd CCCBTrajManager::computebj(int N, int dim, 
        const Eigen::VectorXd &pi, const Eigen::VectorXd &pf){
    assert(pi.size()==dim && pf.size()==dim);
    // bj = (N)*dim x 1
    Eigen::VectorXd bj = Eigen::VectorXd::Zero(N*dim);
    bj.segment(0,dim) = -pi;
    bj.segment(dim,dim) = 2.*pi;
    bj.segment(2*dim,dim) = -pi;
    bj.segment((N-3)*dim,dim) = pf;
    bj.segment((N-2)*dim,dim) = -2.*pf;
    bj.segment((N-1)*dim,dim) = pf;
    return bj;
}

/* -------------------------------------------- */
/*      sepcial math utils useful for CCCB      */
/* -------------------------------------------- */
Eigen::MatrixXd CCCBTrajManager::stack1dMatDim(int dim, const Eigen::MatrixXd &A1d){
    // stack matrix for 1d in dim
    // e.g. A1d = [a11, a12, ..] -> Ad = [a11, 0, .., a12, 0, .. ]
    //            [a21, a22, ..]         [ 0 , a11,.., 0 , a12,..]
    if(dim>1){
        // A1d = m x n
        // Ad = (m*dim) x (n*dim) matrix
        Eigen::MatrixXd repmat = Eigen::MatrixXd::Identity(dim,dim);
        return rossy_utils::kroneckerProduct(A1d, repmat);
    }
    else
        return A1d;
}

