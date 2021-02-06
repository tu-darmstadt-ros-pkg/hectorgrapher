#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_gnc_2d.h"
#include "Eigen/Core"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_gnc_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_gnc_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/common/ceres_solver_options.h"
//#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
//#include "cartographer/common/ceres_solver_options.h"
//#include "cartographer/common/lua_parameter_dictionary.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

//const int MAX_RETRIES = 1;

CeresScanMatcherGnc2D::CeresScanMatcherGnc2D(
    const proto::CeresScanMatcherOptions2D& options)
    : CeresScanMatcher2D(options),
      gnc_weight_update(options.gnc_options_2d(),
                        ceres_solver_options_.function_tolerance,
                        ceres_solver_options_.gradient_tolerance),
      logging_callback(),
      max_retries(options.gnc_options_2d().max_retries()) {


  if (options.gnc_options_2d().use_gnc()) { // only use gnc when enabled in cfg

//    // TODO remove after eval:
//    ceres_solver_options_.callbacks.push_back(&logging_callback);
//    ceres_solver_options_.update_state_every_iteration = true;
//    LOG(INFO) << "ADDING CALLBACK";
//    logging_callback.set_gnc(true);

//    ceres_solver_options_.use_inner_iterations = true;
//    ceres_solver_options_.function_tolerance = 1e-10;
//    ceres_solver_options_.parameter_tolerance /= 100;  // change to 1e-10
    ceres_solver_options_.callbacks.push_back(&gnc_weight_update);
    ceres_solver_options_.min_linear_solver_iterations = 80;
//    gnc_weight_update.set_ceres_solver_options(&(this->ceres_solver_options_));
  }
}

CeresScanMatcherGnc2D::~CeresScanMatcherGnc2D() = default;

GncIterationCallback* CeresScanMatcherGnc2D::get_gnc_callback() {
  return &gnc_weight_update;
}

void CeresScanMatcherGnc2D::newMatch(unsigned long new_size) const {
  gnc_weight_update.newMatch(new_size);
}

void CeresScanMatcherGnc2D::newMatch(unsigned long new_size,
                                     const float start_convexity) const {
  gnc_weight_update.newMatch(new_size, start_convexity);
}

void CeresScanMatcherGnc2D::Match(const Eigen::Vector2d& target_translation,
                                  const transform::Rigid2d& initial_pose_estimate,
                                  const sensor::PointCloud& point_cloud,
                                  const Grid2D& grid,
                                  transform::Rigid2d* const pose_estimate,
                                  ceres::Solver::Summary* const summary) const {
  if (options_.gnc_options_2d().use_gnc()) {
    const unsigned long new_size = point_cloud.size();
    CeresScanMatcherGnc2D::newMatch(new_size);
    int retries_count = 0;
    transform::Rigid2d start_pose_estimate = *pose_estimate;
    while (retries_count < max_retries) {
      ++retries_count;
//      LOG(INFO) << "Retries: " << retries_count << "/" << max_retries;
      CeresScanMatcher2D::Match(target_translation, initial_pose_estimate,
                                point_cloud, grid, pose_estimate, summary);
      if ((summary->termination_type == ceres::USER_SUCCESS ||
          summary->FullReport().find("Parameter tolerance reached.")
          != std::string::npos) && retries_count < max_retries) {  // extra check for retries_count to not call newMatch unnecessary
//        // Reset initial Pose estimate?
//        *pose_estimate = transform::Rigid2d({
//            start_pose_estimate.translation().x(),
//            start_pose_estimate.translation().y()},
//            start_pose_estimate.rotation().angle());
        CeresScanMatcherGnc2D::newMatch(new_size, sqrt(
            gnc_weight_update.get_non_convexity_inc_factor()));
        LOG(INFO) << "Restarting GNC Scan Matching, #" << retries_count;
      } else {
//        for (int i = 0; i < gnc_weight_update.get_weights()->size(); i++) {
//          LOG(INFO) << gnc_weight_update.get_weights()->at(i);
//        }
//        LOG(INFO) << "End convexity: " << gnc_weight_update.get_non_convexity();
        break;  // optimization successful
      }
    }
  } else {
    CeresScanMatcher2D::Match(target_translation, initial_pose_estimate,
                              point_cloud, grid, pose_estimate, summary);
  }
}

void CeresScanMatcherGnc2D::Evaluate(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid, double* cost,
    std::vector<double>* residuals, std::vector<double>* jacobians) const {
  const unsigned long new_size = point_cloud.size();
  CeresScanMatcherGnc2D::newMatch(new_size, 0);
  CeresScanMatcher2D::Evaluate(target_translation, initial_pose_estimate,
                               point_cloud, grid, cost, residuals, jacobians);
}

void CeresScanMatcherGnc2D::setupProblem(
    const Eigen::Vector2d& target_translation,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    double* ceres_pose_estimate, ceres::Problem* problem) const {
  CHECK_GT(options_.occupied_space_weight(), 0.);

//  // TODO remove after eval:
//  logging_callback.set_ceres_pose_estimate_ptr(ceres_pose_estimate);

  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      if (options_.gnc_options_2d().use_gnc()) {
        problem->AddResidualBlock(
            CreateOccupiedSpaceGncCostFunction2D(
                options_.occupied_space_weight() /
                    std::sqrt(static_cast<double>(point_cloud.size())),
                point_cloud, grid, &gnc_weight_update),
            nullptr /* loss function */, ceres_pose_estimate);
      } else {
        problem->AddResidualBlock(CreateOccupiedSpaceCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid), nullptr /* loss function */, ceres_pose_estimate);
      }
      break;
    case GridType::TSDF:
//      unsigned long new_size = point_cloud.size();
//      gnc_weight_update.newMatch(new_size);
      if (options_.gnc_options_2d().use_gnc()) {
        problem->AddResidualBlock(
            CreateTSDFMatchGncCostFunction2D(  // TODO rm unused parameters
                options_.empty_space_cost(),
                options_.occupied_space_weight() /
                std::sqrt(static_cast<double>(point_cloud.size())),
                point_cloud, static_cast<const TSDF2D&>(grid),
                &gnc_weight_update),
            nullptr /* loss function */, ceres_pose_estimate);
      } else {
        problem->AddResidualBlock(
            CreateTSDFMatchCostFunction2D(
                options_.empty_space_cost(), options_.occupied_space_weight() /
                                             std::sqrt(
                                                 static_cast<double>(point_cloud.size())),
                point_cloud, static_cast<const TSDF2D &>(grid) ), nullptr /* loss function */,
            ceres_pose_estimate);
      }
      break;
  }
  CHECK_GE(options_.translation_weight(), 0.);
  problem->AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
          options_.translation_weight(), options_.translation_weight_vertical(),
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
