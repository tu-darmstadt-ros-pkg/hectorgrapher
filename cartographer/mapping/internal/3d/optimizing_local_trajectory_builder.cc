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
#include "cartographer/mapping/internal/3d/scan_matching/scan_matching_optimization_problem.h"
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
      motion_model_(CreateMotionModel("identity")),  // TODO use config
      motion_filter_(options.motion_filter_options()),
      map_update_enabled_(true),
      use_scan_matching_(true),
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
  motion_model_->AddIMUData(imu_data);
}

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  motion_model_->AddOdometryData(odometry_data);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  CHECK_GT(range_data_in_tracking.ranges.size(), 0);

  PointCloudSet point_cloud_set;
  point_cloud_set.time = range_data_in_tracking.time;
  point_cloud_set.origin = range_data_in_tracking.origin;
  point_cloud_set.original_cloud = range_data_in_tracking.ranges;
  point_cloud_set.width = range_data_in_tracking.width;
  point_cloud_set.min_point_timestamp = std::numeric_limits<float>::max();
  point_cloud_set.max_point_timestamp = std::numeric_limits<float>::min();
  for (const auto& hit : range_data_in_tracking.ranges) {
    if (hit.position.hasNaN()) continue;
    const Eigen::Vector3f delta = hit.position - range_data_in_tracking.origin;
    const float range = delta.norm();
    if (range >= options_.min_range() && range <= options_.max_range()) {
      point_cloud_set.points.push_back(hit);
      if (hit.time > point_cloud_set.max_point_timestamp) {
        point_cloud_set.max_point_timestamp = hit.time;
      }
      if (hit.time < point_cloud_set.min_point_timestamp) {
        point_cloud_set.min_point_timestamp = hit.time;
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
  CHECK(motion_model_->isInitialized());
  CHECK(motion_model_->HasDataUntil(t));
  control_points_.push_back(
      ControlPoint{t, motion_model_->ExtrapolateState(t)});
}

void OptimizingLocalTrajectoryBuilder::AddControlPoint(common::Time t,
                                                       double dT, double dR,
                                                       double dt) {
  if (control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
            .initialize_map_orientation_with_imu()) {
      Eigen::Quaterniond g = Eigen::Quaterniond::Identity();
      LOG(INFO) << "TODO estimate direction of gravity: g "
                << g.vec().transpose();
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
  // Check for initialization, if necessary and possible initialize
  if (initial_data_time_ == common::FromUniversal(0)) {
    bool initialized = false;
    while (!point_cloud_data_.empty() &&
           motion_model_->HasDataUntil(point_cloud_data_.front().EndTime()) &&
           !initialized) {
      if (motion_model_->HasDataBefore(point_cloud_data_.front().StartTime())) {
        initial_data_time_ = point_cloud_data_.front().time;
        motion_model_->initialize(point_cloud_data_.front().time);
        initialized = true;
        AddControlPoint(point_cloud_data_.front().time);
      } else {
        point_cloud_data_.pop_front();
      }
    }
    if (!initialized) return nullptr;
  }
  if (time - initial_data_time_ < initialization_duration_) {
    LOG(INFO) << "No Optimization - not enough time since initialization "
              << common::ToSeconds(time - initial_data_time_) << "\t < "
              << common::ToSeconds(initialization_duration_);
    return nullptr;
  }

  // Calibrate IMU
  if (!imu_calibrated_ &&
      options_.optimizing_local_trajectory_builder_options()
          .calibrate_imu()) {
    CalibrateIMU(imu_data_, gravity_constant_,
                 linear_acceleration_calibration_,
                 angular_velocity_calibration_);
    imu_calibrated_ = true;
  }

  // Add control points
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
            motion_model_->HasDataUntil(point_cloud_set.EndTime())) {
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
    LOG(INFO) << "No control point added.";
    return nullptr;
  }
  //  LOG(INFO)<<"num control points: "<<control_points_.size();
  //  LOG(INFO)<<"num clouds points: "<<point_cloud_data_.size();
  //  for(const auto& cp : control_points_) {
  //    LOG(INFO)<<"cp dt: "<<common::ToSeconds(cp.time - initial_data_time_);
  //  }
  //  for(const auto& pc : point_cloud_data_) {
  //    LOG(INFO)<<"pc dt: "<<common::ToSeconds(pc.time - initial_data_time_);
  //  }
  //  LOG(INFO)<<"dt: "<<common::ToSeconds(point_cloud_data_.back().time -
  //  control_points_.back().time);

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

    ScanMatchingOptimizationProblem matching_problem(options_);
    if (use_scan_matching_) {
      if (options_.optimizing_local_trajectory_builder_options()
              .use_multi_resolution_matching()) {
        tsdf_pyramid_ = {dynamic_cast<const HybridGridTSDF*>(
                             &matching_submap->high_resolution_hybrid_grid()),
                         dynamic_cast<const HybridGridTSDF*>(
                             &matching_submap->low_resolution_hybrid_grid())};
        //        tsdf_pyramid_ = {dynamic_cast<const HybridGridTSDF *>(
        //                             &matching_submap->high_resolution_hybrid_grid())};
      }
      if (options_.optimizing_local_trajectory_builder_options()
              .use_per_point_unwarping()) {
        matching_problem.AddPerPointMatchingResiduals();
      } else {
        matching_problem.AddPerScanMatchingResiduals(
            *active_submaps_.submaps().front(), point_cloud_data_,
            tsdf_pyramid_, control_points_);
      }
    }
    matching_problem.AddIMUResiduals(
        imu_data_, linear_acceleration_calibration_,
        angular_velocity_calibration_, imu_integrator_, control_points_);
    matching_problem.AddOdometryResiduals(odometer_data_, control_points_);

    ceres::Solver::Summary summary;
    matching_problem.Solve(control_points_, &summary);
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
  motion_model_->UpdateState(control_points_.back().state,
                             control_points_.back().time);
  const common::Time time_optimized_pose = control_points_.front().time;
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
                                      point_cloud_data_.front().StartTime() &&
             control_points_.back().time >
                 point_cloud_data_.front().EndTime()) {
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
  return motion_model_->ExtrapolateState(end_time);
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

void OptimizingLocalTrajectoryBuilder::UseScanMatching(bool use_scan_matching) {
  use_scan_matching_ = use_scan_matching;
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
