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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_ADAPTIVE_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_ADAPTIVE_VOXEL_FILTER_H_

#include <bitset>

#include "absl/container/flat_hash_set.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/internal/adaptive_voxel_filter.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {
namespace {
template <typename PointCloudType>
PointCloudType FilterByMaxRange(const PointCloudType& point_cloud,
                                const float max_range) {
  PointCloudType result;
  for (const auto& point : point_cloud) {
    if (point.position.norm() <= max_range) {
      result.push_back(point);
    }
  }
  return result;
}

template <typename PointCloudType>
PointCloudType AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloudType& point_cloud) {
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloudType result = VoxelFilter(options.max_length()).Filter(point_cloud);
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(low_length).Filter(point_cloud);
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 30% off.
      while ((high_length - low_length) / low_length > 3e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloudType candidate =
            VoxelFilter(mid_length).Filter(point_cloud);
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

}  // namespace

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  template <typename PointCloudType>
  PointCloudType Filter(const PointCloudType& point_cloud) const {
    return AdaptivelyVoxelFiltered(
        options_, FilterByMaxRange(point_cloud, options_.max_range()));
  }

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_ADAPTIVE_VOXEL_FILTER_H_
