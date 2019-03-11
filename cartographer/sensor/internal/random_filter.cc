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

#include "cartographer/sensor/internal/random_filter.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

PointCloud RandomFilter::Filter(const PointCloud& point_cloud) {
  PointCloud results;

  return results;
}

TimedPointCloud RandomFilter::Filter(const TimedPointCloud& timed_point_cloud) {
  TimedPointCloud results;

  return results;
}

std::vector<TimedPointCloudOriginData::RangeMeasurement> RandomFilter::Filter(
    const std::vector<TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements) {
  std::vector<TimedPointCloudOriginData::RangeMeasurement> results;

  return results;
}

}  // namespace sensor
}  // namespace cartographer
