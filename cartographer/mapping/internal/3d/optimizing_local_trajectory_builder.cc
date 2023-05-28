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
#include "cartographer/sensor/internal/sampling_filter.h"
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
      motion_model_(CreateMotionModel(
          options.optimizing_local_trajectory_builder_options()
              .motion_model())),
      motion_filter_insertion_(options.motion_filter_options()),
      map_update_enabled_(true),
      use_scan_matching_(true),
      watches_("OptimizingLocalTrajectoryBuilder"),
      debug_logger_("test_log.csv") {}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const sensor::ImuData& imu_data) {
  watches_.GetWatch("AddImuData").Start();
  motion_model_->AddIMUData(imu_data);
  watches_.GetWatch("AddImuData").Stop();
}

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  watches_.GetWatch("AddOdometryData").Start();
  motion_model_->AddOdometryData(odometry_data);
  watches_.GetWatch("AddOdometryData").Stop();
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  CHECK_GT(range_data_in_tracking.ranges.size(), 0);
  watches_.GetWatch("add_range_data").Start();
  point_cloud_queue_.emplace_back(range_data_in_tracking, options_);
  std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult> res =
      MaybeOptimize(range_data_in_tracking.time);
  watches_.GetWatch("add_range_data").Stop();
  PrintLoggingData();
  return res;
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
  CHECK(motion_model_->isInitialized());
  CHECK(motion_model_->HasDataUntil(t));
  control_points_.push_back(
      ControlPoint{t, motion_model_->ExtrapolateState(t), dT, dR, dt});
}

void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (control_points_.empty()) {
    return;
  }
  while (ct_window_horizon_ <
             control_points_.back().time - control_points_.front().time &&
         std::next(control_points_.begin())->time <
             point_cloud_active_data_.front().time +
                 common::FromSeconds(
                     point_cloud_active_data_.front().min_point_timestamp)) {
    debug_logger_.AddEntry(control_points_.front());
    control_points_.pop_front();
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
        Eigen::Vector3d(velocity.data());
    control_point.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  // Check for initialization, if necessary and possible initialize
  if (initial_data_time_ == common::FromUniversal(0)) {
    bool initialized = false;
    while (!point_cloud_queue_.empty() &&
           motion_model_->HasDataUntil(point_cloud_queue_.front().EndTime()) &&
           !initialized) {
      if (motion_model_->HasDataBefore(
              point_cloud_queue_.front().StartTime())) {
        initial_data_time_ = point_cloud_queue_.front().time;
        motion_model_->initialize(point_cloud_queue_.front().time);
        initialized = true;
        AddControlPoint(point_cloud_queue_.front().time);
//        LOG(INFO)<<"first cp "<<control_points_.front().state.DebugString();
        point_cloud_active_data_.push_back(point_cloud_queue_.front());
        point_cloud_queue_.pop_front();
      } else {
        point_cloud_queue_.pop_front();
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

  // Add control points
  bool added_control_point = false;
  switch (options_.optimizing_local_trajectory_builder_options()
              .control_point_sampling()) {
    case proto::CONSTANT: {
      while (motion_model_->HasDataUntil(control_points_.back().time +
                                         ct_window_rate_)) {
        AddControlPoint(control_points_.back().time + ct_window_rate_);
        added_control_point = true;
      }
      break;
    }
    case proto::SYNCED_WITH_RANGE_DATA: {
      if (options_.optimizing_local_trajectory_builder_options()
              .use_scan_preunwarping()) {
        while (
            !point_cloud_queue_.empty() &&
            motion_model_->HasDataUntil(point_cloud_queue_.front().EndTime())) {
          AddControlPoint(point_cloud_queue_.front().time);
          watches_.GetWatch("unwarp").Start();
          Unwarp(point_cloud_queue_.front());
          watches_.GetWatch("unwarp").Stop();
          point_cloud_active_data_.push_back(point_cloud_queue_.front());
          point_cloud_queue_.pop_front();
          added_control_point = true;
        }
      } else {
        while (!point_cloud_queue_.empty() &&
               motion_model_->HasDataUntil(point_cloud_queue_.front().time)) {
          AddControlPoint(point_cloud_queue_.front().time);
          point_cloud_active_data_.push_back(point_cloud_queue_.front());
          point_cloud_queue_.pop_front();
          added_control_point = true;
        }
      }
      break;
    }
    case proto::ADAPTIVE: {
      LOG(ERROR) << "ADAPTIVE sampling scheme not implemented";
      // TOOD sample control points based on motion model

      //      const double max_delta_translation =
      //          options_.optimizing_local_trajectory_builder_options()
      //              .sampling_max_delta_translation();
      //      const double max_delta_rotation =
      //          options_.optimizing_local_trajectory_builder_options()
      //              .sampling_max_delta_rotation();
      //      const double max_delta_time =
      //          options_.optimizing_local_trajectory_builder_options()
      //              .sampling_max_delta_time();
      //      const double min_delta_time =
      //          options_.optimizing_local_trajectory_builder_options()
      //              .sampling_min_delta_time();

      break;
    }
    default:
      LOG(FATAL) << "Unsupported control_point_sampling type.";
  }
  if (!added_control_point) {
    //    LOG(INFO) << "No control point added.";
    return nullptr;
  }
  //    LOG(INFO)<<"num control points: "<<control_points_.size();
  //    LOG(INFO)<<"num clouds points: "<<point_cloud_data_.size();
  //    for(const auto& cp : control_points_) {
  //      LOG(INFO)<<"cp dt: "<<common::ToSeconds(cp.time - initial_data_time_);
  //    }
  //    for(const auto& pc : point_cloud_data_) {
  //      LOG(INFO)<<"pc dt: "<<common::ToSeconds(pc.time - initial_data_time_);
  //    }
  //    LOG(INFO)<<"dt: "<<common::ToSeconds(point_cloud_data_.back().time -
  //    control_points_.back().time);

  if (!active_submaps_.submaps().empty()) {
    watches_.GetWatch("optimization").Start();
    std::shared_ptr<const Submap3D> matching_submap =
        active_submaps_.submaps().front();
    // We assume the map is always aligned with the direction of gravity
    CHECK(matching_submap->local_pose().inverse().rotation().isApprox(
        Eigen::Quaterniond::Identity(), 1e-8));
    // We transform the states in 'control_points_' in place to be in the submap
    // frame as expected by the CostFunctor. This is reverted after solving the
    // optimization problem.
    TransformStates(matching_submap->local_pose().inverse());

    ScanMatchingOptimizationProblem matching_problem(options_);
    if (use_scan_matching_) {
      if (options_.optimizing_local_trajectory_builder_options()
              .use_multi_resolution_matching()) {
        tsdf_pyramid_ = {dynamic_cast<const HybridGridTSDF*>(
                             &matching_submap->high_resolution_hybrid_grid()),
                         dynamic_cast<const HybridGridTSDF*>(
                             &matching_submap->low_resolution_hybrid_grid())};
      }
      if (options_.optimizing_local_trajectory_builder_options()
              .use_per_point_unwarping()) {
        matching_problem.AddPerPointMatchingResiduals();
      } else {
        matching_problem.AddPerScanMatchingResiduals(
            *active_submaps_.submaps().front(), point_cloud_active_data_,
            tsdf_pyramid_, control_points_);
      }
    }

    ceres::Solver::Summary summary;

    //    LOG(INFO)<<"before opt";
    //    for(const auto& c : control_points_) {
    //      c.state.Print();
    //    }
    matching_problem.Solve(control_points_, &summary);
    //    LOG(INFO)<<"after opt";
    //    for(const auto& c : control_points_) {
    //      c.state.Print();
    //    }
    //    LOG(INFO) << summary.FullReport();
    // The optimized states in 'control_points_' are in the submap frame and
    // we transform them in place to be in the local SLAM frame again.
    TransformStates(matching_submap->local_pose());
    watches_.GetWatch("optimization").Stop();
  }

  motion_model_->UpdateState(control_points_.back().state,
                             control_points_.back().time);
  const transform::Rigid3d optimized_pose =
      control_points_.front().state.ToRigid();
  const common::Time time_optimized_pose = control_points_.front().time;
  sensor::TimedRangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}, point_cloud_active_data_.front().width};

  watches_.GetWatch("unwarp").Start();
  // unwarp and accumulate

  while (!control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
            .control_point_sampling() == proto::SYNCED_WITH_RANGE_DATA) {
      CHECK(control_points_.front().time ==
            point_cloud_active_data_.front().time);
      auto transform_cloud = control_points_.front().state.ToRigid();
      const transform::Rigid3f transform =
          (optimized_pose.inverse() * transform_cloud).cast<float>();
      for (const auto& point :
           point_cloud_active_data_.front().insertion_cloud) {
        accumulated_range_data_in_tracking.returns.push_back(
            (transform * point));
      }
      accumulated_range_data_in_tracking.origin =
          (transform * point_cloud_active_data_.front().origin);
    }
    control_points_.pop_front();
    point_cloud_active_data_.pop_front();
  }

  watches_.GetWatch("unwarp").Stop();

  RemoveObsoleteSensorData();

  return AddAccumulatedRangeData(time_optimized_pose, optimized_pose,
                                 accumulated_range_data_in_tracking);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const transform::Rigid3d& optimized_pose,
    const sensor::TimedRangeData& range_data_in_tracking) {
  if (range_data_in_tracking.returns.empty()) {
    return nullptr;
  }

  watches_.GetWatch("adaptive_voxel_filter").Start();
  sensor::TimedRangeData filtered_range_data_in_local =
      sensor::TransformTimedRangeData(range_data_in_tracking,
                                      optimized_pose.cast<float>());

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::TimedPointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    watches_.GetWatch("adaptive_voxel_filter").Stop();
    return nullptr;
  }
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::TimedPointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    watches_.GetWatch("adaptive_voxel_filter").Stop();
    return nullptr;
  }
  watches_.GetWatch("adaptive_voxel_filter").Stop();

  const Eigen::Quaterniond& gravity_alignment = optimized_pose.rotation();
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, optimized_pose,
      gravity_alignment);

  return absl::make_unique<MatchingResult>(
      MatchingResult{time, optimized_pose, filtered_range_data_in_local,
                     std::move(insertion_result)});
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time,
    const sensor::TimedRangeData&
        filtered_range_data_in_local,  // Inserted into Submap
    const sensor::TimedRangeData&
        filtered_range_data_in_tracking,  // Used to compute histogram
    const sensor::TimedPointCloud&
        high_resolution_point_cloud_in_tracking,  // Added to
                                                  // TrajectoryNode::Data
    const sensor::TimedPointCloud&
        low_resolution_point_cloud_in_tracking,  // Added to
                                                 // TrajectoryNode::Data
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_insertion_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }

  watches_.GetWatch("compute_histogram").Start();
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
  watches_.GetWatch("compute_histogram").Stop();

  watches_.GetWatch("map_insertion").Start();
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      map_update_enabled_
          ? active_submaps_.InsertData(
                filtered_range_data_in_local, local_from_gravity_aligned,
                rotational_scan_matcher_histogram_in_gravity, time)
          : active_submaps_.submaps();
  watches_.GetWatch("map_insertion").Stop();

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
  watches_.PrintAllEveryN(500);
}

void OptimizingLocalTrajectoryBuilder::Unwarp(PointCloudSet& point_cloud_set) {
  Unwarp(point_cloud_set.insertion_cloud, point_cloud_set.time);
  Unwarp(point_cloud_set.scan_matching_cloud, point_cloud_set.time);
}
void OptimizingLocalTrajectoryBuilder::Unwarp(
    sensor::TimedPointCloud& point_cloud, const common::Time& cloud_time) {
  for (size_t idx = 0; idx < point_cloud.size(); ++idx) {
    common::Time t0 = cloud_time;
    common::Time t1 = t0 + common::FromSeconds(point_cloud[idx].time);
    transform::Rigid3d transform = transform::Rigid3d::Rotation(motion_model_->RelativeTransform(t0, t1).rotation());
    point_cloud[idx] = {
        transform.cast<float>() *
            point_cloud_queue_.front().insertion_cloud[idx].position,
        point_cloud_queue_.front().insertion_cloud[idx].time};
  }
}

}  // namespace mapping
}  // namespace cartographer
