#include <gtest/gtest.h>


// HiGHS is designed to solve linear optimization problems of the form
//
// Min (1/2)x^TQx + c^Tx + d subject to L <= Ax <= U; l <= x <= u
//
// where A is a matrix with m rows and n columns, and Q is either zero
// or positive definite. If Q is zero, HiGHS can determine the optimal
// integer-valued solution.
//
// The scalar n is num_col_
// The scalar m is num_row_
//
// The vector c is col_cost_
// The scalar d is offset_
// The vector l is col_lower_
// The vector u is col_upper_
// The vector L is row_lower_
// The vector U is row_upper_
//
// The matrix A is represented in packed vector form, either
// row-wise or column-wise: only its nonzeros are stored
//
// * The number of nonzeros in A is num_nz
//
// * The indices of the nonnzeros in the vectors of A are stored in a_index
//
// * The values of the nonnzeros in the vectors of A are stored in a_value
//
// * The position in a_index/a_value of the index/value of the first
// nonzero in each vector is stored in a_start
//
// Note that a_start[0] must be zero
//
// The matrix Q is represented in packed column form
//
// * The dimension of Q is dim_
//
// * The number of nonzeros in Q is hessian_num_nz
//
// * The indices of the nonnzeros in the vectors of A are stored in q_index
//
// * The values of the nonnzeros in the vectors of A are stored in q_value
//
// * The position in q_index/q_value of the index/value of the first
// nonzero in each column is stored in q_start
//
// Note
//
// * By default, Q is zero. This is indicated by dim_ being initialised to zero.
//
// * q_start[0] must be zero
//
#include <Highs.h>
#include <cassert>

#include "rossy_utils/solvers/qp_solver.hpp"

using std::cout;
using std::endl;

const bool dev_run = true;
const double inf = std::numeric_limits<double>::infinity();
const double double_equal_tolerance = 1e-5;

#define INF 1e8

TEST(QPSOLVERTEST, qpsolvertest ){
    // minimize -x_2 - 3x_3 + (1/2)(2x_1^2 - 2x_1x_3 + 0.2x_2^2 + 2x_3^2)
  // subject to x_1 + x_3 <= 2; x>=0
  Eigen::MatrixXd Q {{2,0,-1},{0,0.2,0},{-1,0,2} };
  Eigen::MatrixXd A {{1,0,1},{-1,0,0},{0,-1,0},{0,0,-1}};
  Eigen::VectorXd q {{0,-1,-3}};
  Eigen::VectorXd b {{2,0,0,0}};
  Eigen::VectorXd x; 
  double ret = rossy_utils::qpprogHiGHS(Q, q,  A, b, x);
  std::cout << "ret = " << ret << std::endl;
}

TEST(QPSOLVERTEST, testSemiDefinite0 ){
  // Test passing/reading and solving the problem qjh
  //
  // minimize -x_2 - 3x_3 + (1/2)(2x_1^2 - 2x_1x_3 + 0.2x_2^2 + 2x_3^2)
  //
  // subject to x_1 + x_3 <= 2; x>=0
  HighsStatus return_status;
  HighsModelStatus model_status;
  double required_objective_function_value;

  HighsModel local_model;
  HighsLp& lp = local_model.lp_;
  HighsHessian& hessian = local_model.hessian_;
  // Start with an unconstrained QP
  lp.model_name_ = "qjh";
  lp.num_col_ = 3;
  lp.num_row_ = 0;
  lp.col_cost_ = {0.0, -1.0, -3.0};
  lp.col_lower_ = {-INF, -INF, -INF};
  lp.col_upper_ = {INF, INF, INF};
  lp.sense_ = ObjSense::kMinimize;
  lp.offset_ = 0;
  hessian.dim_ = lp.num_col_;

   hessian.format_ = HessianFormat::kSquare;
   hessian.start_ = {0, 2, 3, 5};
   hessian.index_ = {0, 2, 1, 0, 2};
   hessian.value_ = {2.0, -1.0, 0.2, -1.0, 2.0};

  // hessian.format_ = HessianFormat::kTriangular;
  // hessian.start_ = {0, 2, 3, 4};
  // hessian.index_ = {0, 2, 1, 2};
  // hessian.value_ = {2.0, -1.0, 0.2, 2.0};

  Highs highs;
  highs.setOptionValue("output_flag", dev_run);
  const HighsInfo& info = highs.getInfo();
  const double& objective_function_value = info.objective_function_value;
  return_status = highs.passModel(local_model);
  assert(return_status == HighsStatus::kOk);
  if (dev_run) highs.writeModel("");
  return_status = highs.run();
  assert(return_status == HighsStatus::kOk);
  required_objective_function_value = -5.50;
  assert(fabs(objective_function_value - required_objective_function_value) <
          double_equal_tolerance);

  if (dev_run) printf("Objective = %g\n", objective_function_value);
  if (dev_run) highs.writeSolution("", kSolutionStylePretty);

  // Now with a constraint
  lp.num_row_ = 1;
  lp.col_lower_ = {0.0, 0.0, 0.0};
  lp.row_lower_ = {-inf};
  lp.row_upper_ = {2};
  lp.a_matrix_.start_ = {0, 1, 1, 2};
  lp.a_matrix_.index_ = {0, 0};
  lp.a_matrix_.value_ = {1.0, 1.0};
  lp.a_matrix_.format_ = MatrixFormat::kColwise;
  return_status = highs.passModel(local_model);
  assert(return_status == HighsStatus::kOk);
  if (dev_run) highs.writeModel("");
  return_status = highs.run();
  assert(return_status == HighsStatus::kOk);
  required_objective_function_value = -5.25;
  assert(fabs(objective_function_value - required_objective_function_value) <
          double_equal_tolerance);

  if (dev_run) printf("Objective = %g\n", objective_function_value);
  if (dev_run) highs.writeSolution("", kSolutionStylePretty);

  // Make the problem infeasible
  return_status = highs.changeColBounds(0, 3, INF);
  assert(return_status == HighsStatus::kOk);
  return_status = highs.changeColBounds(2, 3, INF);
  assert(return_status == HighsStatus::kOk);
  return_status = highs.run();
  assert(return_status == HighsStatus::kOk);

  if (dev_run) highs.writeSolution("", kSolutionStylePretty);
  model_status = highs.getModelStatus();
  if (dev_run)
    printf("Infeasible QP status: %s\n",
           highs.modelStatusToString(model_status).c_str());
  assert(model_status == HighsModelStatus::kInfeasible);

  return_status = highs.clearModel();

  std::string filename;
  for (HighsInt test_k = 0; test_k < 2; test_k++) {
    if (test_k == 0) {
      filename = "/home/jelee/my_ws/TrajOpt/test/testdata/HighsInstances/qjh.mps";
    } else if (test_k == 1) {
      filename = "/home/jelee/my_ws/TrajOpt/test/testdata/HighsInstances/qjh_quadobj.mps";
    } else {
      filename = "/home/jelee/my_ws/TrajOpt/test/testdata/HighsInstances/qjh_qmatrix.mps";
    }

    return_status = highs.readModel(filename);
    assert(return_status == HighsStatus::kOk);
    return_status = highs.run();
    assert(return_status == HighsStatus::kOk);
    assert(fabs(objective_function_value - required_objective_function_value) <
            double_equal_tolerance);
    return_status = highs.clearModel();
  }
}
