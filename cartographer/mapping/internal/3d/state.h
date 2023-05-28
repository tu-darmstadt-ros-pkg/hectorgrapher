
#ifndef CARTOGRAPHER_MAPPING_3D_STATE_H_
#define CARTOGRAPHER_MAPPING_3D_STATE_H_

#include <array>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/sensor/internal/adaptive_voxel_filter.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct State {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // Rotation quaternion as (w, x, y, z).
  std::array<double, 3> velocity;
  std::array<double, 3> angular_velocity{};

  State(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& velocity)
      : translation{{translation.x(), translation.y(), translation.z()}},
        rotation{{rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
        velocity{{velocity.x(), velocity.y(), velocity.z()}} {}

  Eigen::Quaterniond ToQuaternion() const {
    return Eigen::Quaterniond(
        {rotation[0], rotation[1], rotation[2], rotation[3]});
  }

  Eigen::Quaterniond Rotation() const { return ToQuaternion(); }

  Eigen::Vector3d Translation() const {
    return Eigen::Vector3d(translation.data());
  }

  Eigen::Vector3d AngularVelocity() const {
    return Eigen::Vector3d(angular_velocity.data());
  }

  Eigen::Vector3d LinearVelocity() const {
    return Eigen::Vector3d(velocity.data());
  }

  transform::Rigid3d ToRigid() const {
    return {Eigen::Vector3d(translation[0], translation[1], translation[2]),
            ToQuaternion()};
  }
  std::string DebugString() const {return ToRigid().DebugString(); }
};

struct ControlPoint {
  common::Time time;
  State state;
  double translation_ratio = 0.0;
  double rotation_ratio = 0.0;
  double time_ratio = 0.0;
};

struct PointCloudSet {
  common::Time time;
  Eigen::Vector3f origin;
  sensor::TimedPointCloud scan_matching_cloud;
  sensor::TimedPointCloud insertion_cloud;
  size_t width{};
  float min_point_timestamp{};
  float max_point_timestamp{};
  PointCloudSet() = default;
  PointCloudSet(const sensor::TimedPointCloudData& range_data_in_tracking,
                const proto::LocalTrajectoryBuilderOptions3D& options) {
    time = range_data_in_tracking.time;
    origin = range_data_in_tracking.origin;
    width = range_data_in_tracking.width;
    min_point_timestamp = std::numeric_limits<float>::max();
    max_point_timestamp = std::numeric_limits<float>::min();
    sensor::TimedPointCloud pre_filtered_cloud;
    const float max_range_squared = std::pow(options.max_range(), 2);
    const float min_range_squared = std::pow(options.min_range(), 2);
    for (const auto& hit : range_data_in_tracking.ranges) {
      if (hit.position.hasNaN()) continue;
      const Eigen::Vector3f delta =
          hit.position - range_data_in_tracking.origin;
      const float range = delta.squaredNorm();
      if (range >= min_range_squared && range <= max_range_squared) {
        pre_filtered_cloud.push_back(hit);
        if (hit.time > max_point_timestamp) {
          max_point_timestamp = hit.time;
        }
        if (hit.time < min_point_timestamp) {
          min_point_timestamp = hit.time;
        }
      }
    }

    //  float pre_length = float(options_.submaps_options().high_resolution()
    //  / 2.0); sensor::TimedPointCloud pre_voxel_filtered_cloud;
    //  sensor::VoxelFilter pre_voxel_filter(pre_length);
    //  pre_voxel_filtered_cloud =
    //      pre_voxel_filter.Filter(pre_filtered_cloud);

    sensor::VoxelFilter high_resolution_voxel_filter(
        options.submaps_options().high_resolution() / 2.f);
    insertion_cloud = high_resolution_voxel_filter.Filter(pre_filtered_cloud);
    sensor::AdaptiveVoxelFilter scan_cloud_adaptive_voxel_filter(
        options.high_resolution_adaptive_voxel_filter_options());
    scan_matching_cloud =
        scan_cloud_adaptive_voxel_filter.Filter(insertion_cloud);

    //  LOG_EVERY_N(INFO, 50)<<"raw res cloud size
    //  "<<range_data_in_tracking.ranges.size(); LOG_EVERY_N(INFO, 50)<<"pre
    //  cloud size "<<pre_filtered_cloud.size(); LOG_EVERY_N(INFO, 50)<<"high
    //  res cloud size "<<point_cloud_set.high_resolution_cloud.size();
    //  LOG_EVERY_N(INFO, 50)<<"low res cloud size
    //  "<<point_cloud_set.low_resolution_cloud.size(); LOG_EVERY_N(INFO,
    //  50)<<"match cloud size
    //  "<<point_cloud_set.scan_matching_cloud.size();
  };

  common::Time StartTime() const {
    return time + common::FromSeconds(min_point_timestamp);
  };

  common::Time EndTime() const {
    return time + common::FromSeconds(max_point_timestamp);
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_STATE_H_
