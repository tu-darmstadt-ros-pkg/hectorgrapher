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

#ifndef CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_

#include <array>
#include <chrono>
#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/debug_logger.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/motion_model/motion_model_factory.h"
#include "cartographer/mapping/internal/3d/state.h"
#include "cartographer/mapping/internal/3d/stop_watch.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/adaptive_voxel_filter.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {


// Batches up some sensor data and optimizes them in one go to get a locally
// consistent trajectory.
class OptimizingLocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  };

  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit OptimizingLocalTrajectoryBuilder(
      const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~OptimizingLocalTrajectoryBuilder();

  OptimizingLocalTrajectoryBuilder(const OptimizingLocalTrajectoryBuilder&) =
      delete;
  OptimizingLocalTrajectoryBuilder& operator=(
      const OptimizingLocalTrajectoryBuilder&) = delete;

  void AddImuData(const sensor::ImuData& imu_data);

  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data_in_tracking);

  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

  void SetMapUpdateEnabled(bool map_update_enabled);
  void UseScanMatching(bool use_scan_matching);

 private:
  void AddControlPoint(common::Time t);
  void AddControlPoint(common::Time t, double dT, double dR, double dt);

  void RemoveObsoleteSensorData();

  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const transform::Rigid3d& pose_observation,
      const sensor::TimedRangeData& range_data_in_tracking);

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time,
      const sensor::TimedRangeData& filtered_range_data_in_local,
      const sensor::TimedRangeData& filtered_range_data_in_tracking,
      const sensor::TimedPointCloud& high_resolution_point_cloud_in_tracking,
      const sensor::TimedPointCloud& low_resolution_point_cloud_in_tracking,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  void TransformStates(const transform::Rigid3d& transform);
  std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
  MaybeOptimize(common::Time time);

  void PrintLoggingData();

  const proto::LocalTrajectoryBuilderOptions3D options_;
  const ceres::Solver::Options ceres_solver_options_;
  mapping::ActiveSubmaps3D active_submaps_;
  common::Time initial_data_time_;

  std::deque<ControlPoint> control_points_;
  std::deque<PointCloudSet> point_cloud_data_;

  common::Duration ct_window_horizon_;
  common::Duration ct_window_rate_;
  common::Duration initialization_duration_;

  std::unique_ptr<MotionModel> motion_model_;
  MotionFilter motion_filter_insertion_;
  std::vector<const mapping::HybridGridTSDF*> tsdf_pyramid_;
  bool map_update_enabled_;
  bool use_scan_matching_;

  //Logging
  StopWatchManger watches_;
  DebugLogger debug_logger_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_OPTIMIZING_LOCAL_TRAJECTORY_BUILDER_H_
