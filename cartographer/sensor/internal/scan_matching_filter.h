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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_SCAN_MATCHING_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_SCAN_MATCHING_FILTER_H_

#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class ScanMatchingFilter {
 public:

  // Returns a voxel filtered copy of 'point_cloud'.
  virtual PointCloud Filter(const PointCloud& point_cloud) = 0;

};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_SCAN_MATCHING_FILTER_H_
