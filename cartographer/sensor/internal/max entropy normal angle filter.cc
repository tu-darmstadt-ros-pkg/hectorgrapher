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

#include "cartographer/sensor/internal/max_entropy_normal_angle_filter.h"

#include <cmath>
#include <vector>

#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

std::vector<float> normalOrientations(const PointCloud& point_cloud) {
  std::vector<float> res;
  return res;
}

// Currently only in 2D!!!
PointCloud MaxEntropyNormalAngleFilter::Filter(const PointCloud& point_cloud) {

  auto angles = normalOrientations(point_cloud);

  PointCloud results;


  return results;
}

}  // namespace sensor
}  // namespace cartographer
