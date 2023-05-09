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

#include "cartographer/mapping/internal/3d/scan_matching/scan_matching_optimization_problem.h"

#include "cartographer/mapping/internal/3d/scan_matching/interpolated_multi_resolution_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_per_point_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/multi_resolution_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_direct_imu_integration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_imu_preintegration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/relative_translation_and_yaw_cost_function.h"

namespace cartographer {
namespace mapping {
ScanMatchingOptimizationProblem::ScanMatchingOptimizationProblem(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options)
    : options_(options),
      ceres_solver_options_(common::CreateCeresSolverOptions(
          options.ceres_scan_matcher_options().ceres_solver_options())) {}

void ScanMatchingOptimizationProblem::Solve(
    std::deque<ControlPoint>& control_points, ceres::Solver::Summary* summary) {
  problem.SetParameterBlockConstant(
      control_points.front().state.translation.data());
  problem.SetParameterBlockConstant(
      control_points.front().state.rotation.data());
  if (options_.optimizing_local_trajectory_builder_options()
          .velocity_in_state()) {
    problem.SetParameterBlockConstant(
        control_points.front().state.velocity.data());
  }

  for (size_t i = 1; i < control_points.size(); ++i) {
    problem.SetParameterization(control_points[i].state.rotation.data(),
                                new ceres::QuaternionParameterization());
  }
  ceres::Solve(ceres_solver_options_, &problem, summary);
}

void ScanMatchingOptimizationProblem::AddPerScanMatchingResiduals(
    const Submap3D& matching_submap,
    const std::deque<PointCloudSet>& point_cloud_data,
    const std::vector<const mapping::HybridGridTSDF*>& tsdf_pyramid,
    std::deque<ControlPoint>& control_points) {
  auto next_control_point = control_points.begin();
  for (auto& point_cloud_set : point_cloud_data) {
    if (point_cloud_set.time > control_points.back().time) {
      LOG(INFO)
          << "Ommitting pointcloud a we do not have enough control points";
      break;
    }
    while (next_control_point->time <= point_cloud_set.time) {
      if (std::next(next_control_point) == control_points.end()) break;
      next_control_point++;
    }
    CHECK(next_control_point != control_points.begin());
    CHECK_LE(std::prev(next_control_point)->time, point_cloud_set.time);
    CHECK_GE(next_control_point->time, point_cloud_set.time);
    const double duration = common::ToSeconds(
        next_control_point->time - std::prev(next_control_point)->time);
    const double interpolation_factor =
        common::ToSeconds(point_cloud_set.time -
                          std::prev(next_control_point)->time) /
        duration;
    if (options_.optimizing_local_trajectory_builder_options()
            .use_multi_resolution_matching()) {
      CHECK(matching_submap.high_resolution_hybrid_grid().GetGridType() ==
            GridType::TSDF)
          << "Multi resolution matching only available for GridType::TSDF.";
      if (options_.optimizing_local_trajectory_builder_options()
                  .high_resolution_grid_weight() > 0.0 &&
          !point_cloud_set.high_resolution_filtered_points.empty()) {
        if (interpolation_factor == 0.0 || interpolation_factor == 1.0) {
          ControlPoint& control_point = (interpolation_factor == 0.0)
                                            ? *std::prev(next_control_point)
                                            : *next_control_point;
          //          LOG(INFO) << "Discrete Residual";
          AddMultiResolutionTSDFSpaceCostFunction3DResidual(
              control_point, point_cloud_set.high_resolution_filtered_points,
              tsdf_pyramid);
        } else {
          LOG(INFO) << "Continuous Residual";
          AddInterpolatedMultiResolutionTSDFSpaceCostFunction3DResidual(
              *std::prev(next_control_point), *next_control_point,
              point_cloud_set.high_resolution_filtered_points, tsdf_pyramid,
              interpolation_factor);
        }
      }
    } else {
      LOG(ERROR) << "Only Multi Resolution Matching available";
    }
  }
}

void ScanMatchingOptimizationProblem::AddPerPointMatchingResiduals() {
  LOG(ERROR) << "AddPerPointMatchingResiduals not implemented";
}

void ScanMatchingOptimizationProblem::AddIMUResiduals(
    const std::deque<sensor::ImuData>& imu_data,
    Eigen::Transform<double, 3, Eigen::Affine> linear_acceleration_calibration,
    Eigen::Transform<double, 3, Eigen::Affine> angular_velocity_calibration,
    const std::unique_ptr<ImuIntegrator>& imu_integrator,
    std::deque<ControlPoint>& control_points) {
  if (options_.optimizing_local_trajectory_builder_options()
              .translation_weight() == 0.0 &&
      options_.optimizing_local_trajectory_builder_options()
              .velocity_weight() == 0.0 &&
      options_.optimizing_local_trajectory_builder_options()
              .rotation_weight() == 0.0) {
    return;
  }
  CHECK(options_.optimizing_local_trajectory_builder_options()
            .velocity_in_state())
      << "IMU residuals require velocity_in_state to be enabled.";  // todo(kdaun)
                                                                    // Add
                                                                    // rotation
                                                                    // residual
                                                                    // which
                                                                    // works
                                                                    // without
                                                                    // velocity
  Eigen::Vector3d gravity = gravity_constant_ * Eigen::Vector3d::UnitZ();
  switch (
      options_.optimizing_local_trajectory_builder_options().imu_cost_term()) {
    case proto::DIRECT: {
      for (size_t i = 1; i < control_points.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                PredictionDirectImuIntegrationCostFunctor, 9, 3, 3, 4, 3, 3, 4>(
                new PredictionDirectImuIntegrationCostFunctor(
                    options_.optimizing_local_trajectory_builder_options()
                        .translation_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .velocity_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .rotation_weight(),
                    common::ToSeconds(control_points[i].time -
                                      control_points[i - 1].time),
                    imu_data, linear_acceleration_calibration,
                    angular_velocity_calibration, control_points[i - 1].time,
                    control_points[i].time,
                    options_.optimizing_local_trajectory_builder_options()
                        .imu_integrator())),
            nullptr, control_points[i - 1].state.translation.data(),
            control_points[i - 1].state.velocity.data(),
            control_points[i - 1].state.rotation.data(),
            control_points[i].state.translation.data(),
            control_points[i].state.velocity.data(),
            control_points[i].state.rotation.data());
      }
      break;
    }
    case proto::PREINTEGRATION: {
      auto it = --imu_data.cend();
      while (it->time > control_points.begin()->time) {
        CHECK(it != imu_data.cbegin());
        --it;
      }
      for (size_t i = 1; i < control_points.size(); ++i) {
        IntegrateImuWithTranslationResult<double> result =
            imu_integrator->IntegrateIMU(imu_data, control_points[i - 1].time,
                                         control_points[i].time, &it);

        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                PredictionImuPreintegrationCostFunctor, 9, 3, 3, 4, 3, 3, 4>(
                new PredictionImuPreintegrationCostFunctor(
                    options_.optimizing_local_trajectory_builder_options()
                        .translation_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .velocity_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .rotation_weight(),
                    common::ToSeconds(control_points[i].time -
                                      control_points[i - 1].time),
                    result, gravity)),
            nullptr, control_points[i - 1].state.translation.data(),
            control_points[i - 1].state.velocity.data(),
            control_points[i - 1].state.rotation.data(),
            control_points[i].state.translation.data(),
            control_points[i].state.velocity.data(),
            control_points[i].state.rotation.data());
      }
      break;
    }
    default:
      LOG(FATAL) << "Unknown proto::IMUCostTerm";
  }
}

void ScanMatchingOptimizationProblem::AddOdometryResiduals(
    const std::deque<sensor::OdometryData>& odometer_data_list,
    std::deque<ControlPoint>& control_points) {
  if (odometer_data_list.size() > 1) {
    transform::TransformInterpolationBuffer interpolation_buffer;
    for (const auto& odometer_data : odometer_data_list) {
      interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
    }
    for (size_t i = 1; i < control_points.size(); ++i) {
      // Only add constraints for this range data if  we have bracketing data
      // from the odometer.
      if (!(interpolation_buffer.earliest_time() <=
                control_points[i - 1].time &&
            control_points[i].time <= interpolation_buffer.latest_time())) {
        continue;
      }
      const transform::Rigid3d previous_odometer_pose =
          interpolation_buffer.Lookup(control_points[i - 1].time);
      const transform::Rigid3d current_odometer_pose =
          interpolation_buffer.Lookup(control_points[i].time);
      const transform::Rigid3d delta_pose =
          current_odometer_pose.inverse() * previous_odometer_pose;
      const double delta_time = common::ToSeconds(control_points[i].time -
                                                  control_points[i - 1].time);

      double residual_translation_weight =
          options_.optimizing_local_trajectory_builder_options()
              .odometry_translation_weight();
      double residual_rotation_weight =
          options_.optimizing_local_trajectory_builder_options()
              .odometry_rotation_weight();
      if (options_.optimizing_local_trajectory_builder_options()
              .use_adaptive_odometry_weights()) {
        double translation_distance = std::abs(delta_pose.translation().norm());
        double rotation_distance =
            std::abs(delta_pose.rotation().angularDistance(
                Eigen::Quaterniond::Identity()));
        const double translation_normalization =
            options_.optimizing_local_trajectory_builder_options()
                .odometry_translation_normalization() *
            delta_time;
        const double rotation_normalization =
            options_.optimizing_local_trajectory_builder_options()
                .odometry_rotation_normalization() *
            delta_time;
        residual_translation_weight =
            options_.optimizing_local_trajectory_builder_options()
                .odometry_translation_weight() /
            sqrt(translation_distance + translation_normalization);
        residual_rotation_weight =
            options_.optimizing_local_trajectory_builder_options()
                .odometry_rotation_weight() /
            sqrt(rotation_distance + rotation_normalization);
      }
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<RelativeTranslationAndYawCostFunction,
                                          6, 3, 4, 3, 4>(
              new RelativeTranslationAndYawCostFunction(
                  residual_translation_weight, residual_rotation_weight,
                  delta_pose)),
          nullptr, control_points[i - 1].state.translation.data(),
          control_points[i - 1].state.rotation.data(),
          control_points[i].state.translation.data(),
          control_points[i].state.rotation.data());
    }
  }
}

void ScanMatchingOptimizationProblem::
    AddMultiResolutionTSDFSpaceCostFunction3DResidual(
        ControlPoint& control_point, const sensor::TimedPointCloud& point_cloud,
        const std::vector<const HybridGridTSDF*>& tsdf_pyramid) {
  problem.AddResidualBlock(
      scan_matching::MultiResolutionTSDFSpaceCostFunction3D<
          sensor::TimedPointCloud>::
          CreateAutoDiffCostFunction(
              options_.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, tsdf_pyramid),
      nullptr, control_point.state.translation.data(),
      control_point.state.rotation.data());
}

void ScanMatchingOptimizationProblem::
    AddInterpolatedMultiResolutionTSDFSpaceCostFunction3DResidual(
        ControlPoint& previous_control_point, ControlPoint& next_control_point,
        const sensor::TimedPointCloud& point_cloud,
        const std::vector<const HybridGridTSDF*>& tsdf_pyramid,
        const double& interpolation_factor) {
  problem.AddResidualBlock(
      scan_matching::InterpolatedMultiResolutionTSDFSpaceCostFunction3D<
          sensor::TimedPointCloud>::
          CreateAutoDiffCostFunction(
              options_.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, tsdf_pyramid, interpolation_factor),
      nullptr, previous_control_point.state.translation.data(),
      previous_control_point.state.rotation.data(),
      next_control_point.state.translation.data(),
      next_control_point.state.rotation.data());
}

}  // namespace mapping
}  // namespace cartographer
