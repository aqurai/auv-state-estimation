/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "convex-optimizer.h"

#include <iostream>
#include <stdint.h>

#include <ceres/ceres.h>

#include "state-measurement-history.h"

struct CostFunc {

  CostFunc(StateHistoryManager *shm, int64_t start_ms) {
    shm_ = shm;
    start_ms_ = start_ms;
  }

  template <typename T>
  bool operator()(const T * const P, T* e) const {
    EKFParams p;
    p.param_hdg_offset_deg = P[0];
    p.param_thrust_K = P[1];
    p.param_drag_coef_X = P[2];
    p.param_drag_coef_Y = P[3];
    p.param_stream_vel_x = P[4];
    p.param_stream_vel_y = P[5];
    e[0] = shm_->evaluateChainUpdateError(start_ms_, p);
    return true;
  }

  StateHistoryManager *shm_;
  int64_t start_ms_;
};

double *parameterOptimize(StateHistoryManager *shm, int64_t start_ms, double * param_block) {

  int64_t tms = shm->getLastStateFrameTimestampMs();

  std::cout << "\nCeres test ms " << tms << "\n";

  double est_theta = 0; //param_block[0];
  double est_stream_x = param_block[1];
  double est_stream_y = param_block[2];
  param_block[0] = 0;

  ceres::Problem problem;

  ceres::CostFunction * cost_function = new ceres::NumericDiffCostFunction<CostFunc, ceres::FORWARD, 1, 6>(new CostFunc(shm, start_ms));
  problem.AddResidualBlock(cost_function, nullptr, param_block);
  problem.SetParameterLowerBound(param_block, 0, -190);
  problem.SetParameterUpperBound(param_block, 0, +190);

  problem.SetParameterLowerBound(param_block, 1, 2.0);
  problem.SetParameterUpperBound(param_block, 1, 14);

  problem.SetParameterLowerBound(param_block, 2, std::max(0.4 * param_block[2], 0.1));
  problem.SetParameterUpperBound(param_block, 2, std::min(1.9 * param_block[2], 5.0));

  problem.SetParameterLowerBound(param_block, 3, std::max(0.4 * param_block[3], 0.00));
  problem.SetParameterUpperBound(param_block, 3, std::min(1.9 * param_block[3], 2.5));

  problem.SetParameterLowerBound(param_block, 4, -2.0);
  problem.SetParameterUpperBound(param_block, 4, 2.0);

  problem.SetParameterLowerBound(param_block, 5, -2.0);
  problem.SetParameterUpperBound(param_block, 5, 2.0);

  //  problem.SetParameterBlockConstant(&est_drag_X);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.logging_type = ceres::SILENT;
  //options.max_solver_time_in_seconds = 5;
  options.max_num_iterations = 200;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << "\n############################################################\n";
  //std::cout << summary.FullReport() << std::endl;
  //  std::cout << "Ceres Estimated theta: " << param_block[0] << "\n";
  //  std::cout << "Ceres Estimated Param K: " << param_block[1] << "\n";
  //  std::cout << "Ceres Estimated drag X: " << param_block[2] << "\n";
  //  std::cout << "Ceres Estimated drag Y: " << param_block[3] << "\n";
  //  std::cout << "Ceres Estimated stream x: " << param_block[4] << "\n";
  //  std::cout << "Ceres Estimated stream y: " << param_block[5] << "\n";
  //std::cout << "\n############################################################\n";

  //  param_block[0] = est_theta;
  //  param_block[1] = est_stream_x;
  //  param_block[2] = est_stream_y;
  return param_block;
}