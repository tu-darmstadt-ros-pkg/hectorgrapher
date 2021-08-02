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

#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder.h"

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/imu_static_calibration.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_multi_resolution_tsdf_per_point_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_multi_resolution_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_per_point_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/multi_resolution_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_direct_imu_integration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_imu_preintegration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/relative_translation_and_yaw_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {
class GravityDirectionCostFunction {
 public:
  GravityDirectionCostFunction(
      const double scaling_factor,
      const Eigen::Quaterniond& gravity_direction_in_tracking)
      : scaling_factor_(scaling_factor),
        gravity_direction_in_tracking_(gravity_direction_in_tracking) {}

  GravityDirectionCostFunction(const GravityDirectionCostFunction&) = delete;
  GravityDirectionCostFunction& operator=(const GravityDirectionCostFunction&) =
      delete;

  template <typename T>
  bool operator()(const T* const orientation, T* residual) const {
    Eigen::Quaternion<T> qorientation(orientation[0], orientation[1],
                                      orientation[2], orientation[3]);

    Eigen::Quaternion<T> delta =
        gravity_direction_in_tracking_.cast<T>() * qorientation;
    residual[0] = scaling_factor_ * transform::GetRoll(delta);
    residual[1] = scaling_factor_ * transform::GetPitch(delta);
    return true;
  }

 private:
  const double scaling_factor_;
  const Eigen::Quaterniond gravity_direction_in_tracking_;
};
void AddMultiResolutionTSDFSpaceCostFunction3DResidual(
    ceres::Problem& problem, ControlPoint& control_point,
    const proto::LocalTrajectoryBuilderOptions3D& options,
    const sensor::TimedPointCloud& point_cloud,
    const std::vector<const HybridGridTSDF*>& tsdf_pyramid) {
  problem.AddResidualBlock(
      scan_matching::MultiResolutionTSDFSpaceCostFunction3D<
          sensor::TimedPointCloud>::
          CreateAutoDiffCostFunction(
              options.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, tsdf_pyramid),
      nullptr, control_point.state.translation.data(),
      control_point.state.rotation.data());
}

void AddInterpolatedMultiResolutionTSDFSpaceCostFunction3DResidual(
    ceres::Problem& problem, ControlPoint& previous_control_point,
    ControlPoint& next_control_point,
    const proto::LocalTrajectoryBuilderOptions3D& options,
    const sensor::TimedPointCloud& point_cloud,
    const std::vector<const HybridGridTSDF*>& tsdf_pyramid,
    const double& interpolation_factor) {
  problem.AddResidualBlock(
      scan_matching::InterpolatedMultiResolutionTSDFSpaceCostFunction3D<
          sensor::TimedPointCloud>::
          CreateAutoDiffCostFunction(
              options.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, tsdf_pyramid, interpolation_factor),
      nullptr, previous_control_point.state.translation.data(),
      previous_control_point.state.rotation.data(),
      next_control_point.state.translation.data(),
      next_control_point.state.rotation.data());
}

}  // namespace

OptimizingLocalTrajectoryBuilder::OptimizingLocalTrajectoryBuilder(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      ceres_solver_options_(common::CreateCeresSolverOptions(
          options.ceres_scan_matcher_options().ceres_solver_options())),
      active_submaps_(options.submaps_options()),
      num_accumulated_(0),
      total_num_accumulated_(0),
      initial_data_time_(common::FromUniversal(0)),
      ct_window_horizon_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_horizon())),
      ct_window_rate_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_rate())),
      initialization_duration_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .initialization_duration())),
      imu_calibrated_(false),
      linear_acceleration_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      angular_velocity_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      motion_filter_(options.motion_filter_options()),
      map_update_enabled_(true),
      num_insertions(0),
      total_insertion_duration(0.0),
      num_optimizations(0),
      total_optimization_duration(0.0),
      debug_logger_("test_log.csv") {
  imu_integrator_ = absl::make_unique<ImuIntegrator>(
      options.optimizing_local_trajectory_builder_options().imu_integrator());
}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    imu_data_.push_back(imu_data);
    return;
  }
  initial_data_time_ = imu_data.time;
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
  imu_data_.push_back(imu_data);
}

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return;
  }
  if (!imu_data_.empty() && (imu_data_.front().time >= odometry_data.time)) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Odometry data dropped to maintain IMU consistency.";
    return;
  }
  odometer_data_.push_back(odometry_data);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  CHECK_GT(range_data_in_tracking.ranges.size(), 0);

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }
  if (odometer_data_.empty()) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Odom not yet initialized.";
    return nullptr;
  }

  if (initial_data_time_ >
      range_data_in_tracking.time +
          common::FromSeconds(range_data_in_tracking.ranges.front().time)) {
    LOG(INFO) << "Not enough data, skipping this cloud.";
    return nullptr;
  }

  if (!odometer_data_.empty() &&
      odometer_data_.front().time >
          range_data_in_tracking.time +
              common::FromSeconds(range_data_in_tracking.ranges.front().time)) {
    LOG(INFO) << "Not enough odom data, skipping this cloud.";
    return nullptr;
  }
  PointCloudSet point_cloud_set;
  point_cloud_set.time = range_data_in_tracking.time;
  point_cloud_set.origin = range_data_in_tracking.origin;
  point_cloud_set.original_cloud = range_data_in_tracking.ranges;
  point_cloud_set.width = range_data_in_tracking.width;
  for (const auto& hit : range_data_in_tracking.ranges) {
    if (hit.position.hasNaN()) continue;
    const Eigen::Vector3f delta = hit.position - range_data_in_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        point_cloud_set.points.push_back(hit);
      }
    }
  }

  auto high_resolution_options =
      options_.high_resolution_adaptive_voxel_filter_options();
  high_resolution_options.set_min_num_points(
      high_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter high_resolution_adaptive_voxel_filter(
      high_resolution_options);
  point_cloud_set.high_resolution_filtered_points =
      high_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);

  auto low_resolution_options =
      options_.low_resolution_adaptive_voxel_filter_options();
  low_resolution_options.set_min_num_points(
      low_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      low_resolution_options);
  point_cloud_set.low_resolution_filtered_points =
      low_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);
  point_cloud_data_.push_back(point_cloud_set);

  ++num_accumulated_;
  ++total_num_accumulated_;

  return MaybeOptimize(range_data_in_tracking.time);
}

void OptimizingLocalTrajectoryBuilder::AddControlPoint(common::Time t) {
  if (control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
        .initialize_map_orientation_with_imu()) {
      Eigen::Quaterniond g = extrapolator_->EstimateGravityOrientation(t);
      LOG(INFO) << "g " << g.vec().transpose();
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), g, Eigen::Vector3d::Zero())});
    } else {
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero())});
    }
  } else {
    if (active_submaps_.submaps().empty()) {
      control_points_.push_back(
          ControlPoint{t, control_points_.back().state});
    } else {
      control_points_.push_back(
          ControlPoint{t, PredictState(control_points_.back().state,
                                       control_points_.back().time, t)});
    }
  }
}

void OptimizingLocalTrajectoryBuilder::AddControlPoint(common::Time t,
                                                       double dT, double dR,
                                                       double dt) {
  if (control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
            .initialize_map_orientation_with_imu()) {
      Eigen::Quaterniond g = extrapolator_->EstimateGravityOrientation(t);
      LOG(INFO) << "g " << g.vec().transpose();
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), g, Eigen::Vector3d::Zero()), dT, dR,
          dt});
    } else {
      control_points_.push_back(ControlPoint{
          t,
          State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                Eigen::Vector3d::Zero()),
          dT, dR, dt});
    }
  } else {
    if (active_submaps_.submaps().empty()) {
      control_points_.push_back(
          ControlPoint{t, control_points_.back().state, dT, dR, dt});
    } else {
      control_points_.push_back(
          ControlPoint{t,
                       PredictState(control_points_.back().state,
                                    control_points_.back().time, t),
                       dT, dR, dt});
    }
  }
}

void OptimizingLocalTrajectoryBuilder::AddPerScanMatchingResiduals(
    ceres::Problem& problem) {
  std::shared_ptr<const Submap3D> matching_submap =
      active_submaps_.submaps().front();
  auto next_control_point = control_points_.begin();
  for (auto& point_cloud_set : point_cloud_data_) {
    if (point_cloud_set.time > control_points_.back().time) break;
    while (next_control_point->time <= point_cloud_set.time) {
      if (std::next(next_control_point) == control_points_.end()) break;
      next_control_point++;
    }
    CHECK(next_control_point != control_points_.begin());
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
      CHECK(matching_submap->high_resolution_hybrid_grid().GetGridType() ==
            GridType::TSDF)
          << "Multi resolution matching only available for GridType::TSDF.";
      if (options_.optimizing_local_trajectory_builder_options()
                  .high_resolution_grid_weight() > 0.0 &&
          !point_cloud_set.high_resolution_filtered_points.empty()) {
        if (interpolation_factor == 0.0 || interpolation_factor == 1.0) {
          ControlPoint& control_point = interpolation_factor == 0.0
                                            ? *std::prev(next_control_point)
                                            : *next_control_point;
          AddMultiResolutionTSDFSpaceCostFunction3DResidual(
              problem, control_point, options_,
              point_cloud_set.high_resolution_filtered_points, tsdf_pyramid_);
        } else {
          AddInterpolatedMultiResolutionTSDFSpaceCostFunction3DResidual(
              problem, *std::prev(next_control_point), *next_control_point,
              options_, point_cloud_set.high_resolution_filtered_points,
              tsdf_pyramid_, interpolation_factor);
        }
      }
    } else {
      switch (matching_submap->high_resolution_hybrid_grid().GetGridType()) {
        case GridType::PROBABILITY_GRID: {
          if (options_.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() > 0.0 &&
              !point_cloud_set.high_resolution_filtered_points.empty()) {
            problem.AddResidualBlock(
                scan_matching::InterpolatedOccupiedSpaceCostFunction3D<
                    sensor::TimedPointCloud>::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.high_resolution_filtered_points
                                    .size())),
                        point_cloud_set.high_resolution_filtered_points,
                        dynamic_cast<const HybridGrid&>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
          }
          break;
        }
        case GridType::TSDF: {
          if (options_.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() > 0.0 &&
              !point_cloud_set.high_resolution_filtered_points.empty()) {
            if (std::prev(next_control_point)->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->high_resolution_hybrid_grid())),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data());
            } else if (next_control_point->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->high_resolution_hybrid_grid())),
                  nullptr, next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            } else {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedTSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->high_resolution_hybrid_grid()),
                          interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            }
          }
          if (options_.optimizing_local_trajectory_builder_options()
                      .low_resolution_grid_weight() > 0.0 &&
              !point_cloud_set.low_resolution_filtered_points.empty()) {
            if (std::prev(next_control_point)->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .low_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set.low_resolution_filtered_points
                                      .size())),
                          point_cloud_set.low_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->low_resolution_hybrid_grid())),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data());
            } else if (next_control_point->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .low_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set.low_resolution_filtered_points
                                      .size())),
                          point_cloud_set.low_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->low_resolution_hybrid_grid())),
                  nullptr, next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            } else {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedTSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .low_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set.low_resolution_filtered_points
                                      .size())),
                          point_cloud_set.low_resolution_filtered_points,
                          dynamic_cast<const HybridGridTSDF&>(
                              matching_submap->low_resolution_hybrid_grid()),
                          interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            }
          }
          break;
        }
        case GridType::NONE:
          LOG(FATAL) << "Gridtype not initialized.";
          break;
      }
    }
  }
}

void OptimizingLocalTrajectoryBuilder::AddPerPointMatchingResiduals(
    ceres::Problem& problem) {
  std::shared_ptr<const Submap3D> matching_submap =
      active_submaps_.submaps().front();
  auto next_control_point = control_points_.begin();
  if (options_.optimizing_local_trajectory_builder_options()
          .use_per_point_unwarping()) {
    for (auto& point_cloud_set : point_cloud_data_) {
      for (size_t subdivision_start_idx = 0;
           subdivision_start_idx <
           point_cloud_set.high_resolution_filtered_points.size();
           subdivision_start_idx +=
           options_.optimizing_local_trajectory_builder_options()
               .num_points_per_subdivision()) {
        size_t subdivision_end_idx = std::min(
            subdivision_start_idx +
                options_.optimizing_local_trajectory_builder_options()
                    .num_points_per_subdivision() -
                1,
            point_cloud_set.high_resolution_filtered_points.size() - 1);
        //        const sensor::TimedRangefinderPoint& point =
        //            point_cloud_set
        //                .high_resolution_filtered_points[subdivision_start_idx];

        double center_timestamp =
            0.5 * (point_cloud_set
                       .high_resolution_filtered_points[subdivision_start_idx]
                       .time +
                   point_cloud_set
                       .high_resolution_filtered_points[subdivision_end_idx]
                       .time);
        common::Time subdivision_time =
            point_cloud_set.time + common::FromSeconds(center_timestamp);
        if (subdivision_time < control_points_.back().time &&
            subdivision_time > control_points_.front().time) {
          while (next_control_point->time <= subdivision_time) {
            if (std::next(next_control_point) == control_points_.end()) break;
            next_control_point++;
          }
          while (std::prev(next_control_point)->time > subdivision_time) {
            if (std::prev(next_control_point) == control_points_.begin()) break;
            next_control_point--;
          }
          CHECK(next_control_point != control_points_.begin());
          CHECK_LE(std::prev(next_control_point)->time, subdivision_time);
          CHECK_GE(next_control_point->time, subdivision_time);
          const double duration = common::ToSeconds(
              next_control_point->time - std::prev(next_control_point)->time);
          const double interpolation_factor = common::Clamp(
              common::ToSeconds(subdivision_time -
                                std::prev(next_control_point)->time) /
                  duration,
              0.0, 1.0);
          if (options_.optimizing_local_trajectory_builder_options()
                  .use_multi_resolution_matching()) {
            problem.AddResidualBlock(
                scan_matching::InterpolatedMultiResolutionTSDFSpaceCostFunction3D<
                    sensor::TimedPointCloud>::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.high_resolution_filtered_points
                                    .size())),
                        sensor::TimedPointCloud(
                            point_cloud_set.high_resolution_filtered_points
                                    .begin() +
                                subdivision_start_idx,
                            point_cloud_set.high_resolution_filtered_points
                                    .begin() +
                                subdivision_end_idx + 1),
                        tsdf_pyramid_, interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
          } else {
            problem.AddResidualBlock(
                scan_matching::InterpolatedTSDFSpaceCostFunction3D<
                    sensor::TimedPointCloud>::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.high_resolution_filtered_points
                                    .size())),

                        sensor::TimedPointCloud(
                            point_cloud_set.high_resolution_filtered_points
                                    .begin() +
                                subdivision_start_idx,
                            point_cloud_set.high_resolution_filtered_points
                                    .begin() +
                                subdivision_end_idx + 1),
                        dynamic_cast<const HybridGridTSDF&>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
          }
        } else {
          //          LOG(INFO)<<"omitting point";
        }
      }
    }
    next_control_point = control_points_.begin();
    if (!options_.optimizing_local_trajectory_builder_options()
             .use_multi_resolution_matching() &&
        options_.optimizing_local_trajectory_builder_options()
                .low_resolution_grid_weight() > 0) {
      for (auto& point_cloud_set : point_cloud_data_) {
        for (const auto& point :
             point_cloud_set.low_resolution_filtered_points) {
          common::Time point_time =
              point_cloud_set.time + common::FromSeconds(point.time);
          static int total_points = 0;
          static int omitted_points_before = 0;
          static int omitted_points_after = 0;
          ++total_points;
          LOG_EVERY_N(INFO, 100000)
              << "Omit before ratio "
              << double(omitted_points_before) / double(total_points);
          LOG_EVERY_N(INFO, 100000)
              << "Omit after ratio "
              << double(omitted_points_after) / double(total_points);
          if (point_time < control_points_.back().time &&
              point_time > control_points_.front().time) {
            while (next_control_point->time <= point_time) {
              if (std::next(next_control_point) == control_points_.end()) break;
              next_control_point++;
            }
            while (std::prev(next_control_point)->time > point_time) {
              if (std::prev(next_control_point) == control_points_.begin())
                break;
              next_control_point--;
            }
            CHECK(next_control_point != control_points_.begin());
            CHECK_LE(std::prev(next_control_point)->time, point_time);
            CHECK_GE(next_control_point->time, point_time);
            const double duration = common::ToSeconds(
                next_control_point->time - std::prev(next_control_point)->time);
            const double interpolation_factor = common::Clamp(
                common::ToSeconds(point_time -
                                  std::prev(next_control_point)->time) /
                    duration,
                0.0, 1.0);

            problem.AddResidualBlock(
                scan_matching::InterpolatedTSDFPerPointSpaceCostFunction3D<
                    sensor::TimedRangefinderPoint>::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .low_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.low_resolution_filtered_points
                                    .size())),
                        point,
                        dynamic_cast<const HybridGridTSDF&>(
                            matching_submap->low_resolution_hybrid_grid()),
                        interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
          } else {
            if (point_time < control_points_.front().time)
              ++omitted_points_before;
            if (point_time > control_points_.back().time) {
              ++omitted_points_after;
            }
          }
        }
      }
    }
  } else {
    for (auto& point_cloud_set : point_cloud_data_) {
      if (point_cloud_set.time <= control_points_.back().time) {
        while (next_control_point->time <= point_cloud_set.time) {
          if (std::next(next_control_point) == control_points_.end()) break;
          next_control_point++;
        }
        CHECK(next_control_point != control_points_.begin());
        CHECK_LE(std::prev(next_control_point)->time, point_cloud_set.time);
        CHECK_GE(next_control_point->time, point_cloud_set.time);
        const double duration = common::ToSeconds(
            next_control_point->time - std::prev(next_control_point)->time);
        const double interpolation_factor =
            common::ToSeconds(point_cloud_set.time -
                              std::prev(next_control_point)->time) /
            duration;
        if (point_cloud_set.low_resolution_filtered_points.empty() ||
            point_cloud_set.low_resolution_filtered_points.empty()) {
          continue;
        }
        if (options_.optimizing_local_trajectory_builder_options()
                .use_multi_resolution_matching()) {
          const std::vector<const HybridGridTSDF*> tsdf_pyramid{
              dynamic_cast<const HybridGridTSDF*>(
                  &matching_submap->high_resolution_hybrid_grid()),
              dynamic_cast<const HybridGridTSDF*>(
                  &matching_submap->low_resolution_hybrid_grid())};
          if (options_.optimizing_local_trajectory_builder_options()
                  .high_resolution_grid_weight() > 0.0) {
            if (std::prev(next_control_point)->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::MultiResolutionTSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          tsdf_pyramid),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data());
            } else if (next_control_point->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::MultiResolutionTSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          tsdf_pyramid),
                  nullptr, next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            } else {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedMultiResolutionTSDFSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          tsdf_pyramid, interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            }
          }
        } else {
          switch (
              matching_submap->high_resolution_hybrid_grid().GetGridType()) {
            case GridType::PROBABILITY_GRID: {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedOccupiedSpaceCostFunction3D<
                      sensor::TimedPointCloud>::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .high_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set
                                      .high_resolution_filtered_points.size())),
                          point_cloud_set.high_resolution_filtered_points,
                          dynamic_cast<const HybridGrid&>(
                              matching_submap->high_resolution_hybrid_grid()),
                          interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
              break;
            }
            case GridType::TSDF: {
              if (options_.optimizing_local_trajectory_builder_options()
                      .high_resolution_grid_weight() > 0.0) {
                if (std::prev(next_control_point)->time ==
                    point_cloud_set.time) {
                  problem.AddResidualBlock(
                      scan_matching::TSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .high_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .high_resolution_filtered_points
                                          .size())),
                              point_cloud_set.high_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->high_resolution_hybrid_grid())),
                      nullptr,
                      std::prev(next_control_point)->state.translation.data(),
                      std::prev(next_control_point)->state.rotation.data());
                } else if (next_control_point->time == point_cloud_set.time) {
                  problem.AddResidualBlock(
                      scan_matching::TSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .high_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .high_resolution_filtered_points
                                          .size())),
                              point_cloud_set.high_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->high_resolution_hybrid_grid())),
                      nullptr, next_control_point->state.translation.data(),
                      next_control_point->state.rotation.data());
                } else {
                  problem.AddResidualBlock(
                      scan_matching::InterpolatedTSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .high_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .high_resolution_filtered_points
                                          .size())),
                              point_cloud_set.high_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->high_resolution_hybrid_grid()),
                              interpolation_factor),
                      nullptr,
                      std::prev(next_control_point)->state.translation.data(),
                      std::prev(next_control_point)->state.rotation.data(),
                      next_control_point->state.translation.data(),
                      next_control_point->state.rotation.data());
                }
              }
              if (options_.optimizing_local_trajectory_builder_options()
                      .low_resolution_grid_weight() > 0.0) {
                if (std::prev(next_control_point)->time ==
                    point_cloud_set.time) {
                  problem.AddResidualBlock(
                      scan_matching::TSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .low_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .low_resolution_filtered_points
                                          .size())),
                              point_cloud_set.low_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->low_resolution_hybrid_grid())),
                      nullptr,
                      std::prev(next_control_point)->state.translation.data(),
                      std::prev(next_control_point)->state.rotation.data());
                } else if (next_control_point->time == point_cloud_set.time) {
                  problem.AddResidualBlock(
                      scan_matching::TSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .low_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .low_resolution_filtered_points
                                          .size())),
                              point_cloud_set.low_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->low_resolution_hybrid_grid())),
                      nullptr, next_control_point->state.translation.data(),
                      next_control_point->state.rotation.data());
                } else {
                  problem.AddResidualBlock(
                      scan_matching::InterpolatedTSDFSpaceCostFunction3D<
                          sensor::TimedPointCloud>::
                          CreateAutoDiffCostFunction(
                              options_.optimizing_local_trajectory_builder_options()
                                      .low_resolution_grid_weight() /
                                  std::sqrt(static_cast<double>(
                                      point_cloud_set
                                          .low_resolution_filtered_points
                                          .size())),
                              point_cloud_set.low_resolution_filtered_points,
                              dynamic_cast<const HybridGridTSDF&>(
                                  matching_submap
                                      ->low_resolution_hybrid_grid()),
                              interpolation_factor),
                      nullptr,
                      std::prev(next_control_point)->state.translation.data(),
                      std::prev(next_control_point)->state.rotation.data(),
                      next_control_point->state.translation.data(),
                      next_control_point->state.rotation.data());
                }
              }
              break;
            }
            case GridType::NONE:
              LOG(FATAL) << "Gridtype not initialized.";
              break;
          }
        }
      }
    }
  }
}

void OptimizingLocalTrajectoryBuilder::AddIMUResiduals(
    ceres::Problem& problem) {
  if(options_.optimizing_local_trajectory_builder_options()
      .translation_weight() == 0.0 &&
      options_.optimizing_local_trajectory_builder_options()
          .velocity_weight() == 0.0 &&
      options_.optimizing_local_trajectory_builder_options()
          .rotation_weight() == 0.0) {
    return;
  }
  CHECK(options_.optimizing_local_trajectory_builder_options().velocity_in_state())<<"IMU residuals require velocity_in_state to be enabled."; //todo(kdaun) Add rotation residual which works without velocity
  Eigen::Vector3d gravity = gravity_constant_ * Eigen::Vector3d::UnitZ();
  switch (
      options_.optimizing_local_trajectory_builder_options().imu_cost_term()) {
    case proto::DIRECT: {
      for (size_t i = 1; i < control_points_.size(); ++i) {
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
                    common::ToSeconds(control_points_[i].time -
                                      control_points_[i - 1].time),
                    imu_data_, linear_acceleration_calibration_,
                    angular_velocity_calibration_, control_points_[i - 1].time,
                    control_points_[i].time,
                    options_.optimizing_local_trajectory_builder_options()
                        .imu_integrator())),
            nullptr, control_points_[i - 1].state.translation.data(),
            control_points_[i - 1].state.velocity.data(),
            control_points_[i - 1].state.rotation.data(),
            control_points_[i].state.translation.data(),
            control_points_[i].state.velocity.data(),
            control_points_[i].state.rotation.data());
      }
      break;
    }
    case proto::PREINTEGRATION: {
      auto it = --imu_data_.cend();
      while (it->time > control_points_.begin()->time) {
        CHECK(it != imu_data_.cbegin());
        --it;
      }
      for (size_t i = 1; i < control_points_.size(); ++i) {
        IntegrateImuWithTranslationResult<double> result =
            imu_integrator_->IntegrateIMU(imu_data_,
                                          control_points_[i - 1].time,
                                          control_points_[i].time, &it);

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
                    common::ToSeconds(control_points_[i].time -
                                      control_points_[i - 1].time),
                    result, gravity)),
            nullptr, control_points_[i - 1].state.translation.data(),
            control_points_[i - 1].state.velocity.data(),
            control_points_[i - 1].state.rotation.data(),
            control_points_[i].state.translation.data(),
            control_points_[i].state.velocity.data(),
            control_points_[i].state.rotation.data());
      }
      break;
    }
    default:
      LOG(FATAL) << "Unknown proto::IMUCostTerm";
  }
}

void OptimizingLocalTrajectoryBuilder::AddOdometryResiduals(
    ceres::Problem& problem) {
  if (odometer_data_.size() > 1) {
    transform::TransformInterpolationBuffer interpolation_buffer;
    for (const auto& odometer_data : odometer_data_) {
      interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
    }
    for (size_t i = 1; i < control_points_.size(); ++i) {
      // Only add constraints for this range data if  we have bracketing data
      // from the odometer.
      if (!(interpolation_buffer.earliest_time() <=
                control_points_[i - 1].time &&
            control_points_[i].time <= interpolation_buffer.latest_time())) {
        continue;
      }
      const transform::Rigid3d previous_odometer_pose =
          interpolation_buffer.Lookup(control_points_[i - 1].time);
      const transform::Rigid3d current_odometer_pose =
          interpolation_buffer.Lookup(control_points_[i].time);
      const transform::Rigid3d delta_pose =
          current_odometer_pose.inverse() * previous_odometer_pose;
      const double delta_time = common::ToSeconds(control_points_[i].time -
                                                  control_points_[i - 1].time);

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

//        const double translation_normalization = 1.0E-3 * delta_time;
//        const double rotation_normalization = 5.0E-3 * delta_time;
        const double translation_normalization = 2.0E-2 * delta_time;
        const double rotation_normalization = 1.0E-1 * delta_time;
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
          nullptr, control_points_[i - 1].state.translation.data(),
          control_points_[i - 1].state.rotation.data(),
          control_points_[i].state.translation.data(),
          control_points_[i].state.rotation.data());
    }
  }
}

void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (control_points_.empty()) {
    return;
  }

  while (ct_window_horizon_ <
             control_points_.back().time - control_points_.front().time &&
         std::next(control_points_.begin())->time <
             point_cloud_data_.front().time +
                 common::FromSeconds(
                     point_cloud_data_.front().original_cloud.front().time)) {
    debug_logger_.AddEntry(control_points_.front());
    control_points_.pop_front();
  }

  while (imu_data_.size() > 1 &&
      (imu_data_[1].time <= control_points_.front().time)) {
    imu_data_.pop_front();
  }

  while (odometer_data_.size() > 1 &&
         (odometer_data_[1].time <= control_points_.front().time)) {
    odometer_data_.pop_front();
  }
}

void OptimizingLocalTrajectoryBuilder::TransformStates(
    const transform::Rigid3d& transform) {
  for (ControlPoint& control_point : control_points_) {
    const transform::Rigid3d new_pose =
        transform * control_point.state.ToRigid();
    const auto& velocity = control_point.state.velocity;
    const Eigen::Vector3d new_velocity =
        transform.rotation() *
        Eigen::Vector3d(velocity[0], velocity[1], velocity[2]);
    control_point.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  if (time - initial_data_time_ < initialization_duration_
      //  ||     time - last_optimization_time_ < optimization_rate_
  ) {
    if (time - initial_data_time_ < initialization_duration_) {
      LOG(INFO) << "No Optimization - not enough time since initialization "
                << common::ToSeconds(time - initial_data_time_) << "\t < "
                << common::ToSeconds(initialization_duration_);
    }
    return nullptr;
  }
  if (odometer_data_.size() < 2) {
    LOG(INFO) << "not enough odom data";
    return nullptr;
  }
  if(control_points_.empty()) {
    //    AddControlPoint(initial_data_time_);
    AddControlPoint(std::max(initial_data_time_, odometer_data_.begin()->time));
  }
  if (!imu_calibrated_ &&
      options_.optimizing_local_trajectory_builder_options()
          .calibrate_imu()) {
    CalibrateIMU(imu_data_, gravity_constant_,
                 linear_acceleration_calibration_,
                 angular_velocity_calibration_);
    imu_integrator_->SetLinearAccelerationCalibration(
        linear_acceleration_calibration_);
    imu_integrator_->SetAngularVelocityCalibration(
        angular_velocity_calibration_);
    imu_calibrated_ = true;
    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    sensor::ImuData debiased_imu_data = imu_data_.front();
    debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
    debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
    extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
        ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
        options_.imu_gravity_time_constant(), debiased_imu_data);
    for(size_t i = 1; i < imu_data_.size(); ++i) {
      debiased_imu_data = imu_data_[i];
      debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
      debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
      extrapolator_->AddImuData(debiased_imu_data);
    }
  }

  bool added_control_point = false;

  switch (options_.optimizing_local_trajectory_builder_options()
              .control_point_sampling()) {
    case proto::CONSTANT: {
      while (control_points_.back().time + ct_window_rate_ <
          odometer_data_.back().time) {
        AddControlPoint(control_points_.back().time + ct_window_rate_);
        added_control_point = true;
      }
      break;
    }
    case proto::SYNCED_WITH_RANGE_DATA: {
      for (auto& point_cloud_set : point_cloud_data_) {
        if ((control_points_.back().time < point_cloud_set.time) &&
            (point_cloud_set.time < imu_data_.back().time)) {
          AddControlPoint(point_cloud_set.time);
          added_control_point = true;
        }
      }
      break;
    }
    case proto::ADAPTIVE: {
      if (odometer_data_.size() > 1) {
        const double max_delta_translation =
            options_.optimizing_local_trajectory_builder_options()
                .sampling_max_delta_translation();
        const double max_delta_rotation =
            options_.optimizing_local_trajectory_builder_options()
                .sampling_max_delta_rotation();
        const double max_delta_time =
            options_.optimizing_local_trajectory_builder_options()
                .sampling_max_delta_time();
        const double min_delta_time =
            options_.optimizing_local_trajectory_builder_options()
                .sampling_min_delta_time();

        transform::TransformInterpolationBuffer interpolation_buffer;
        for (const auto& odometer_data : odometer_data_) {
          interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
        }
        common::Time candidate_time = control_points_.back().time;
        while (candidate_time < interpolation_buffer.latest_time()) {
          double translation_ratio = 0.0;
          double rotation_ratio = 0.0;
          double time_ratio = 0.0;
          candidate_time = interpolation_buffer.LookupUntilDelta(
              control_points_.back().time, max_delta_translation,
              max_delta_rotation, max_delta_time, translation_ratio,
              rotation_ratio, time_ratio);
          if (common::ToSeconds(candidate_time -
              control_points_.back().time) <
              min_delta_time) {
            candidate_time = control_points_.back().time +
                common::FromSeconds(min_delta_time);
          }
          if (candidate_time < interpolation_buffer.latest_time()) {
            if (!control_points_.empty()) {
//              LOG(INFO) << "Add CP "
//                        << common::ToSeconds(candidate_time -
//                                             control_points_.back().time);
            }
            AddControlPoint(candidate_time, translation_ratio, rotation_ratio,
                            time_ratio);
            added_control_point = true;
          }
        }
      }
      break;
    }
    default:
      LOG(FATAL) << "Unsupported control_point_sampling type.";
  }

  if (!added_control_point) {
    return nullptr;
  }

  ceres::Problem problem;
  if (!active_submaps_.submaps().empty()) {
    std::shared_ptr<const Submap3D> matching_submap =
        active_submaps_.submaps().front();
    // We assume the map is always aligned with the direction of gravity
    CHECK(matching_submap->local_pose().inverse().rotation().isApprox(
        Eigen::Quaterniond::Identity(), 1e-8));
    // We transform the states in 'control_points_' in place to be in the
    // submap frame as expected by the OccupiedSpaceCostFunctor. This is
    // reverted after solving the optimization problem.
    TransformStates(matching_submap->local_pose().inverse());

    if (options_.optimizing_local_trajectory_builder_options()
            .use_multi_resolution_matching()) {
      tsdf_pyramid_ = {dynamic_cast<const HybridGridTSDF*>(
                           &matching_submap->high_resolution_hybrid_grid()),
                       dynamic_cast<const HybridGridTSDF*>(
                           &matching_submap->low_resolution_hybrid_grid())};
    }
    if (options_.optimizing_local_trajectory_builder_options()
            .use_per_point_unwarping()) {
      AddPerPointMatchingResiduals(problem);
    } else {
      AddPerScanMatchingResiduals(problem);
    }
    AddIMUResiduals(problem);
    AddOdometryResiduals(problem);

    problem.SetParameterBlockConstant(
        control_points_.front().state.translation.data());
    problem.SetParameterBlockConstant(
        control_points_.front().state.rotation.data());
    if(options_.optimizing_local_trajectory_builder_options().velocity_in_state()) {
      problem.SetParameterBlockConstant(
          control_points_.front().state.velocity.data());
    }

    for (size_t i = 1; i < control_points_.size(); ++i) {
      problem.SetParameterization(control_points_[i].state.rotation.data(),
                                  new ceres::QuaternionParameterization());

    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_solver_options_, &problem, &summary);
    ++num_optimizations;
    total_optimization_duration += summary.total_time_in_seconds;
    PrintLoggingData();
    //    LOG(INFO) << summary.FullReport();
    // The optimized states in 'control_points_' are in the submap frame and
    // we transform them in place to be in the local SLAM frame again.
    TransformStates(matching_submap->local_pose());
  }

  num_accumulated_ = 0;
  const transform::Rigid3d optimized_pose =
      control_points_.front().state.ToRigid();
  const common::Time time_optimized_pose = control_points_.front().time;
  extrapolator_->AddPose(control_points_.front().time, optimized_pose);
  sensor::TimedRangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}, point_cloud_data_.front().width};

  if (active_submaps_.submaps().empty()) {
    // To initialize the empty map we add all available range data assuming
    // zero motion.
    auto control_points_iterator = control_points_.begin();
    for (auto& point_cloud_set : point_cloud_data_) {
      if (point_cloud_set.time < control_points_.back().time) {
        while (control_points_iterator->time <= point_cloud_set.time) {
          ++control_points_iterator;
        }
        CHECK(control_points_iterator != control_points_.begin())
            << "Delta "
            << common::ToSeconds(point_cloud_set.time -
                                 control_points_iterator->time);
        CHECK(control_points_iterator != control_points_.end());
        auto transform_cloud = InterpolateTransform(
            std::prev(control_points_iterator)->state.ToRigid(),
            control_points_iterator->state.ToRigid(),
            std::prev(control_points_iterator)->time,
            control_points_iterator->time, point_cloud_set.time);
        const transform::Rigid3f transform =
            (optimized_pose.inverse() * transform_cloud).cast<float>();
        for (const auto& point : point_cloud_set.original_cloud) {
          accumulated_range_data_in_tracking.returns.push_back(
              (transform * point));
        }
        accumulated_range_data_in_tracking.origin =
            (transform * point_cloud_set.origin);
      }
    }
  } else {
    if (options_.optimizing_local_trajectory_builder_options()
            .use_per_point_unwarping()) {
      CHECK(control_points_.front().time <=
            point_cloud_data_.front().StartTime());

      auto next_control_point = control_points_.begin();
      bool first_point = true;
      while (ct_window_horizon_ < control_points_.back().time -
                                      point_cloud_data_.front().StartTime()) {
        for (const auto& point : point_cloud_data_.front().original_cloud) {
          if (point.position.hasNaN()) {
            accumulated_range_data_in_tracking.returns.push_back(point);
            continue;
          }
          common::Time point_time =
              point_cloud_data_.front().time + common::FromSeconds(point.time);
          while (next_control_point->time <= point_time) {
            if (std::next(next_control_point) == control_points_.end()) break;
            next_control_point++;
          }
          while (std::prev(next_control_point)->time > point_time) {
            if (std::prev(next_control_point) == control_points_.begin()) break;
            next_control_point--;
          }
          CHECK(next_control_point != control_points_.begin())
              << "dt c0-point \t "
              << common::ToSeconds(next_control_point->time - point_time);
          CHECK_LE(std::prev(next_control_point)->time, point_time);
          CHECK_GE(next_control_point->time, point_time);

          auto transform_cloud = InterpolateTransform(
              std::prev(next_control_point)->state.ToRigid(),
              next_control_point->state.ToRigid(),
              std::prev(next_control_point)->time, next_control_point->time,
              point_time);
          const transform::Rigid3f transform =
              (optimized_pose.inverse() * transform_cloud).cast<float>();
          accumulated_range_data_in_tracking.returns.push_back(transform *
                                                               point);
          if (first_point) {
            accumulated_range_data_in_tracking.origin =
                (transform * point_cloud_data_.front().origin);
            first_point = false;
          }
        }
        point_cloud_data_.pop_front();
      }
    } else {
      CHECK(control_points_.front().time <= point_cloud_data_.front().time);
      while (ct_window_horizon_ - ct_window_rate_ <
             control_points_.back().time - point_cloud_data_.front().time) {
        while (std::next(control_points_.begin())->time <
               point_cloud_data_.front().time) {
          debug_logger_.AddEntry(control_points_.front());
          control_points_.pop_front();
        }
        CHECK(std::next(control_points_.begin()) != control_points_.end());
        auto transform_cloud = InterpolateTransform(
            control_points_.begin()->state.ToRigid(),
            std::next(control_points_.begin())->state.ToRigid(),
            control_points_.begin()->time,
            std::next(control_points_.begin())->time,
            point_cloud_data_.front().time);
        const transform::Rigid3f transform =
            (optimized_pose.inverse() * transform_cloud).cast<float>();
        for (const auto& point : point_cloud_data_.front().original_cloud) {
          accumulated_range_data_in_tracking.returns.push_back(transform *
                                                               point);
        }
        accumulated_range_data_in_tracking.origin =
            (transform * point_cloud_data_.front().origin);
        point_cloud_data_.pop_front();
      }
    }
  }

  RemoveObsoleteSensorData();

  return AddAccumulatedRangeData(time_optimized_pose, optimized_pose,
                                 accumulated_range_data_in_tracking);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const transform::Rigid3d& optimized_pose,
    const sensor::TimedRangeData& range_data_in_tracking) {
  if (range_data_in_tracking.returns.empty()) {
    //    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  sensor::TimedRangeData filtered_range_data_in_tracking = {
      range_data_in_tracking.origin,
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.returns),
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.misses)};

  if (filtered_range_data_in_tracking.returns.empty()) {
    //    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }
//  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
//      filtered_range_data_in_tracking, optimized_pose.cast<float>());
  sensor::TimedRangeData filtered_range_data_in_local =
      sensor::TransformTimedRangeData(range_data_in_tracking,
                                      optimized_pose.cast<float>());

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::TimedPointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::TimedPointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  const Eigen::Quaterniond gravity_alignment = optimized_pose.rotation();
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, optimized_pose,
      gravity_alignment);

  return absl::make_unique<MatchingResult>(MatchingResult{
      time, optimized_pose, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time,
    const sensor::TimedRangeData& filtered_range_data_in_local,
    const sensor::TimedRangeData& filtered_range_data_in_tracking,
    const sensor::TimedPointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::TimedPointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::ToPointCloud(sensor::TransformTimedPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>()))),
          options_.rotational_histogram_size());

  const Eigen::Quaterniond local_from_gravity_aligned =
      pose_estimate.rotation() * gravity_alignment.inverse();
  if (!map_update_enabled_) {
    LOG(WARNING) << "Map Update Disabled!";
  }

  double t_before_insert = common::GetThreadCpuTimeSeconds();
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      map_update_enabled_
          ? active_submaps_.InsertData(
                filtered_range_data_in_local, local_from_gravity_aligned,
                rotational_scan_matcher_histogram_in_gravity, time)
          : active_submaps_.submaps();
  double t_after_insert = common::GetThreadCpuTimeSeconds();
  ++num_insertions;
  total_insertion_duration += t_after_insert - t_before_insert;

  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const mapping::TrajectoryNode::Data>(
          mapping::TrajectoryNode::Data{
              time,
              gravity_alignment,
              {},  // 'filtered_point_cloud' is only used in 2D.
              sensor::ToPointCloud(high_resolution_point_cloud_in_tracking),
              sensor::ToPointCloud(low_resolution_point_cloud_in_tracking),
              rotational_scan_matcher_histogram_in_gravity,
              pose_estimate}),
      std::move(insertion_submaps)});
}

State OptimizingLocalTrajectoryBuilder::PredictState(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  //return start_state;
  bool predict_odom = true;
  if (predict_odom) {
    return PredictStateOdom(start_state, start_time, end_time);
  }

  switch (
      options_.optimizing_local_trajectory_builder_options().imu_integrator()) {
    CHECK(options_.optimizing_local_trajectory_builder_options().velocity_in_state())<<"IMU based state propagation requires velocity_in_state to be enabled.";
    case proto::IMUIntegrator::EULER:
      return PredictStateEuler(start_state, start_time, end_time);
    case proto::IMUIntegrator::RK4:
      return PredictStateRK4(start_state, start_time, end_time);
    default:
      LOG(FATAL) << "Unsupported imu integrator type.";
  }
}

State OptimizingLocalTrajectoryBuilder::PredictStateRK4(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }
  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateStateWithGravity(imu_data_, start_state,
                                                 start_time, end_time, &it);
  return State(result.delta_translation, result.delta_rotation,
               result.delta_velocity);
}

State OptimizingLocalTrajectoryBuilder::PredictStateEuler(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }

  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateIMU(imu_data_, start_time, end_time, &it);

  const Eigen::Quaterniond start_rotation(
      start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
      start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * result.delta_rotation;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

  const Eigen::Vector3d position =
      Eigen::Map<const Eigen::Vector3d>(start_state.translation.data()) +
      delta_time_seconds *
          Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data());
  const Eigen::Vector3d velocity =
      Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data()) +
      start_rotation * result.delta_velocity -
      gravity_constant_ * delta_time_seconds * Eigen::Vector3d::UnitZ();

  return State(position, orientation, velocity);
}

State OptimizingLocalTrajectoryBuilder::PredictStateOdom(
  const State& start_state, const common::Time start_time,
  const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }


  transform::TransformInterpolationBuffer interpolation_buffer;
  for (const auto& odometer_data : odometer_data_) {
    interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
  }
//  if (!(interpolation_buffer.earliest_time() <=
//    control_points_[i - 1].time &&
//    control_points_[i].time <= interpolation_buffer.latest_time())) {
//    continue;
//  }


  const common::Time earliest_time =
      interpolation_buffer.earliest_time();
  const common::Time latest_time =
      interpolation_buffer.latest_time();
  transform::Rigid3d previous_odometer_pose;
  if (interpolation_buffer.Has(start_time)) {
    previous_odometer_pose = interpolation_buffer.Lookup(start_time);
  } else if (start_time < earliest_time) {
    LOG(WARNING) << "start_time out of buffer, too early "<<common::ToSeconds(earliest_time-start_time);
    previous_odometer_pose = interpolation_buffer.Lookup(earliest_time);
  } else if (start_time > latest_time) {
    LOG(WARNING) << "start_time out of buffer, too late "<<common::ToSeconds(start_time-latest_time);
    previous_odometer_pose = interpolation_buffer.Lookup(latest_time);
  }
  transform::Rigid3d current_odometer_pose;
  if (interpolation_buffer.Has(end_time)) {
    current_odometer_pose = interpolation_buffer.Lookup(end_time);
  } else if (end_time < earliest_time) {
    current_odometer_pose = interpolation_buffer.Lookup(earliest_time);
    LOG(WARNING) << "end_time out of buffer, too early "<<common::ToSeconds(earliest_time-end_time);
  } else if (end_time > latest_time) {
    current_odometer_pose = interpolation_buffer.Lookup(latest_time);
    LOG(WARNING) << "end_time out of buffer, too late "<<common::ToSeconds(end_time-latest_time)<<" ratio "<<common::ToSeconds(end_time-latest_time)/common::ToSeconds(end_time-start_time);
  }

  const transform::Rigid3d delta_pose =
    current_odometer_pose.inverse() * previous_odometer_pose;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

  const Eigen::Vector3d position =
    Eigen::Map<const Eigen::Vector3d>(start_state.translation.data()) + delta_pose.translation();
  const Eigen::Vector3d velocity = (1.0/delta_time_seconds) * delta_pose.translation();

  const Eigen::Quaterniond start_rotation(
    start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
    start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * delta_pose.rotation();

  return State(position, orientation, velocity);
}


void OptimizingLocalTrajectoryBuilder::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  LOG(WARNING)
      << "OptimizingLocalTrajectoryBuilder::RegisterMetrics not implemented";
}

void OptimizingLocalTrajectoryBuilder::SetMapUpdateEnabled(
    bool map_update_enabled) {
  map_update_enabled_ = map_update_enabled;
}

void OptimizingLocalTrajectoryBuilder::PrintLoggingData() {
  if (num_optimizations > 0) {
    LOG_EVERY_N(INFO, 100) << "Optimization - Avg: "
                           << total_optimization_duration / num_optimizations
                           << "\t total: " << total_optimization_duration;
  }
  if (num_insertions > 0) {
    LOG_EVERY_N(INFO, 100) << "Insertion - Avg: "
                           << total_insertion_duration / num_insertions
                           << "\t total: " << total_insertion_duration;
  }
}

}  // namespace mapping
}  // namespace cartographer
