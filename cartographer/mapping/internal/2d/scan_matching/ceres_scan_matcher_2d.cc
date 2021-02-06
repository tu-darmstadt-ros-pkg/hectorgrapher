/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "text_format.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
      parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_translation_weight_vertical(
      parameter_dictionary->GetDouble("translation_weight_vertical"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  options.set_empty_space_cost(parameter_dictionary->GetDouble("empty_space_cost"));
  if (parameter_dictionary->HasKey("gnc_options_2d")) {
    LOG(INFO) << "Creating GNC OPTIONS";
    *options.mutable_gnc_options_2d() =
        cartographer::mapping::scan_matching::CreateGncOptions2D(
        parameter_dictionary->GetDictionary("gnc_options_2d").get());
  }
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
    const proto::CeresScanMatcherOptions2D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())),
      logging_callback() {  // TODO REMOVE

  // TODO remove after eval:
//  ceres_solver_options_.update_state_every_iteration = true;
//  LOG(INFO) << "ADDING CALLBACK";
//  ceres_solver_options_.callbacks.push_back(&logging_callback);
//  logging_callback.set_gnc(false);

//  std::string a;
//  google::protobuf::TextFormat::PrintToString(options.ceres_solver_options(), &a);
//  LOG(INFO) << "Got Solver Options:" << a;
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
//  ceres_solver_options_.function_tolerance = 1e-12;
//  ceres_solver_options_.gradient_tolerance = 1e-14;
//  ceres_solver_options_.parameter_tolerance = 1e-12;

//    LOG(INFO) << "Creating_________";
//    std::unique_ptr<ceres::IterationCallback> gnc_callback;
//    gnc_callback.reset(new GncIterationCallback());

//    LOG(INFO) << "inserting__________";
//    ceres_solver_options_.callbacks.push_back(&gnc_callback);
//    LOG(INFO) << "inserting done__________";

}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  // this function is called depending on the Class of the scan matcher at run
  // time, either GNC or Normal scan matching problem setup
  setupProblem(target_translation, initial_pose_estimate, point_cloud, grid,
               &ceres_pose_estimate[0], &problem);
  ceres::Solve(ceres_solver_options_, &problem, summary);
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Evaluate(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid, double* cost,
    std::vector<double>* residuals, std::vector<double>* jacobians) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  setupProblem(target_translation, initial_pose_estimate, point_cloud, grid,
               &ceres_pose_estimate[0], &problem);
  problem.Evaluate(ceres::Problem::EvaluateOptions(), cost, residuals,
                   jacobians, NULL);
}

void CeresScanMatcher2D::setupProblem(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    double* ceres_pose_estimate, ceres::Problem* problem) const {
  CHECK_GT(options_.occupied_space_weight(), 0.);

//  // TODO remove after eval:
//  logging_callback.set_ceres_pose_estimate_ptr(ceres_pose_estimate);

  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      problem->AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      problem->AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
              options_.empty_space_cost(),
              options_.occupied_space_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D&>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
      break;
  }
  CHECK_GE(options_.translation_weight(), 0.);
  problem->AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(),
          options_.translation_weight_vertical(),
          target_translation, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GE(options_.rotation_weight(), 0.);
  problem->AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
