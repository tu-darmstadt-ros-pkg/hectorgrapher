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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

PointCloud VoxelFilter::Filter(const PointCloud& point_cloud) {
  PointCloud results;
  for (const RangefinderPoint& point : point_cloud) {
    auto it_inserted =
        voxel_set_.insert(IndexToKey(GetCellIndex(point.position)));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

TimedPointCloud VoxelFilter::Filter(const TimedPointCloud& timed_point_cloud) {
  TimedPointCloud results;
  for (const TimedRangefinderPoint& point : timed_point_cloud) {
    auto it_inserted =
        voxel_set_.insert(IndexToKey(GetCellIndex(point.position)));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

std::vector<TimedPointCloudOriginData::RangeMeasurement> VoxelFilter::Filter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements) {
  std::vector<TimedPointCloudOriginData::RangeMeasurement> results;
  for (const auto& range_measurement : range_measurements) {
    auto it_inserted = voxel_set_.insert(
        IndexToKey(GetCellIndex(range_measurement.point_time.position)));
    if (it_inserted.second) {
      results.push_back(range_measurement);
    }
  }
  return results;
}

VoxelFilter::KeyType VoxelFilter::IndexToKey(const Eigen::Array3i& index) {
  KeyType k_0(static_cast<uint32>(index[0]));
  KeyType k_1(static_cast<uint32>(index[1]));
  KeyType k_2(static_cast<uint32>(index[2]));
  return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
}

Eigen::Array3i VoxelFilter::GetCellIndex(const Eigen::Vector3f& point) const {
  Eigen::Array3f index = point.array() / resolution_;
  return Eigen::Array3i(common::RoundToInt(index.x()),
                        common::RoundToInt(index.y()),
                        common::RoundToInt(index.z()));
}




}  // namespace sensor
}  // namespace cartographer
