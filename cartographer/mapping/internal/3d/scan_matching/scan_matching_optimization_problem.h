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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OPTIMIZATION_OPTIMIZATION_PROBLEM_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OPTIMIZATION_OPTIMIZATION_PROBLEM_3D_H_

#include <deque>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/scan_matching/absolute_rigid_cost_function.h"
#include "cartographer/mapping/internal/3d/state.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_options.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "ceres/ceres.h"
namespace cartographer {
namespace mapping {
class ScanMatchingOptimizationProblem {
 public:
  ScanMatchingOptimizationProblem(
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options);

  void Solve(std::deque<ControlPoint>& control_points,
             ceres::Solver::Summary* summary);

  void AddPerScanMatchingResiduals(
      const Submap3D& matching_submap,
      const std::deque<PointCloudSet>& point_cloud_data,
      const std::vector<const mapping::HybridGridTSDF*>& tsdf_pyramid,
      std::deque<ControlPoint>& control_points);

  void AddPerPointMatchingResiduals();

  void AddIMUResiduals(
      const std::deque<sensor::ImuData>& imu_data,
      Eigen::Transform<double, 3, Eigen::Affine>
          linear_acceleration_calibration,
      Eigen::Transform<double, 3, Eigen::Affine> angular_velocity_calibration,
      const std::unique_ptr<ImuIntegrator>& imu_integrator,
      std::deque<ControlPoint>& control_points);

  void AddOdometryResidual(ControlPoint& control_point);

  void AddOdometryResiduals(
      const std::deque<sensor::OdometryData>& odometer_data_list,
      std::deque<ControlPoint>& control_points);

 private:
  void AddMultiResolutionTSDFSpaceCostFunction3DResidual(
      ControlPoint& control_point, const sensor::TimedPointCloud& point_cloud,
      const std::vector<const HybridGridTSDF*>& tsdf_pyramid);

  void AddInterpolatedMultiResolutionTSDFSpaceCostFunction3DResidual(
      ControlPoint& previous_control_point, ControlPoint& next_control_point,
      const sensor::TimedPointCloud& point_cloud,
      const std::vector<const HybridGridTSDF*>& tsdf_pyramid,
      const double& interpolation_factor);

  ceres::Problem problem;
  const proto::LocalTrajectoryBuilderOptions3D options_;
  const ceres::Solver::Options ceres_solver_options_;
  double gravity_constant_ = 9.80665;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_OPTIMIZATION_OPTIMIZATION_PROBLEM_3D_H_
