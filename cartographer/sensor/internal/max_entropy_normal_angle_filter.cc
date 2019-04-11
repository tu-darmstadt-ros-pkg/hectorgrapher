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
#include "cartographer/sensor/internal/normal_estimation_helper.h"

#include <cmath>
#include <vector>
#include <set>
#include <random>
#include "Eigen/Core"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {


// Currently only in 2D!!!
PointCloud MaxEntropyNormalAngleFilter::Filter(const PointCloud& point_cloud) {
  std::random_device rd;
  std::mt19937 gen(rd());

  size_t number_bins = 20; // TODO make configurable
  auto histogram = GenerateNormalHistogram(point_cloud, number_bins);
  PointCloud results;

  for(size_t i = 0; i < number_of_points_; ++i) {
    size_t histogram_index = i % number_bins;

    auto bin = histogram[histogram_index];
    while (!bin.empty()) {
      histogram_index = (histogram_index + 1) % number_bins;
      bin = histogram[histogram_index];
    }

    std::uniform_int_distribution<> dis(0, bin.size() - 1);
    auto point_index = dis(gen);

    results.push_back(point_cloud[bin[point_index]]);

  }


  return results;
}

}  // namespace sensor
}  // namespace cartographer
