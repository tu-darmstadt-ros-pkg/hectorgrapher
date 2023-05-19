
#ifndef CARTOGRAPHER_MAPPING_3D_STATE_H_
#define CARTOGRAPHER_MAPPING_3D_STATE_H_

#include <array>

#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"
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
  sensor::TimedPointCloud high_resolution_cloud;
  sensor::TimedPointCloud low_resolution_cloud;
  size_t width;
  float min_point_timestamp;
  float max_point_timestamp;

  common::Time StartTime() {
    return time + common::FromSeconds(min_point_timestamp);
  };

  common::Time EndTime() {
    return time + common::FromSeconds(max_point_timestamp);
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_STATE_H_
