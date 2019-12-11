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
  //std::cerr << "Start filter" << std::endl;
  std::random_device rd;
  std::mt19937 gen(rd());

  size_t number_bins = 20; // TODO make configurable
  //std::cerr << "Generate Histogram" << std::endl ;
  auto histogram = GenerateNormalHistogram(point_cloud, number_bins);
  //std::cerr << "Complete Histogram" << std::endl;
  std::vector<int> lengths;
  lengths.resize(number_bins);
  std::transform (histogram.begin(), histogram.end(), lengths.begin(), [](std::vector<size_t> bin){
    return bin.size();
  });

  PointCloud results;

  uint points_to_remove = std::max<int>(0, point_cloud.size() - number_of_points_) ;
  // maximizing the entropy of the distribution over the normal orientations
  for(size_t i = 0; i < points_to_remove; ++i) {
    size_t histogram_index = std::max_element(lengths.begin(),lengths.end()) - lengths.begin();
    auto& bin = histogram[histogram_index];

    std::uniform_int_distribution<> dis(0, bin.size() - 1);
    auto point_index = dis(gen);

    lengths[histogram_index] = lengths[histogram_index] - 1;
    bin.erase(bin.begin() + point_index);
  }

  // add points to result point cloud
  for(size_t i = 0; i < histogram.size(); ++i) {
    auto& bin = histogram[i];
    for(size_t j = 0; j < bin.size(); ++j) {
      results.push_back(point_cloud[bin[j]]);
    }
  }

  //std::cerr << "Complete filter" << std::endl;
  return results;
}

}  // namespace sensor
}  // namespace cartographer
