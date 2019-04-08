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
#include <set>
#include <random>
#include "Eigen/Core"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

Eigen::Vector2f pca(const std::set<size_t>& neighbors, const PointCloud& point_cloud) {
  // Construct mean
  Eigen::Vector2f mean(0,0);
  for(auto index : neighbors){
    mean += point_cloud[index].position.head<2>();
  }
  mean = 1.0/neighbors.size() * mean;

  // Construct cov matrix
  Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
  for(auto index : neighbors){
    Eigen::Vector2f mean_div = point_cloud[index].position.head<2>() - mean;
    cov += mean_div * mean_div.transpose();
  }
  cov = 1.0/(neighbors.size() - 1) * cov;

  // compute eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(cov);
  Eigen::Vector2f eigen_vector = eigen_solver.eigenvectors().col(0);

  return eigen_vector;
}

std::set<size_t> selectNeighborhood(const RangefinderPoint& point, const PointCloud& point_cloud){
  std::set<size_t> neighbors;
  size_t min_number_points = 3; // TODO make configurable
  float ball_radius = 0.25; // TODO make configurable

  while(neighbors.size() < 3){
    for(size_t index = 0; index < point_cloud.size(); ++index){
      auto singlePoint = point_cloud[index];
      auto distance = (point.position - singlePoint.position).norm();
      if(distance <= min_number_points)
        neighbors.insert(index);
    }
    ball_radius = ball_radius + 0.5 * ball_radius;
  }

  return neighbors;
}

Eigen::Vector2f estimateNormal(const RangefinderPoint& point, const PointCloud& point_cloud){
  auto neighbors = selectNeighborhood(point, point_cloud);
  auto normal_candidate = pca(neighbors, point_cloud);
  normal_candidate.normalize();

  float dir = normal_candidate.dot(-point.position.head<2>()); // origin = 0

  return dir > 0 ? normal_candidate : -normal_candidate;
}

float normalOrientation( const RangefinderPoint& point, const PointCloud& point_cloud) {
  auto normal = estimateNormal(point, point_cloud);

  float x = normal[0];
  float y = normal[1];

  float angle = 0;

  if(x > 0 && y >= 0)
    angle = std::atan(y/x);
  else if(x > 0 && y < 0)
    angle = std::atan(y/x) + 2*M_PI;
  else if(x < 0)
    angle = std::atan(y/x) + M_PI;
  else if(x==0 && y > 0)
    angle = M_PI/2;
  else
    angle = 3/2 * M_PI;

  return angle;
}

std::vector<std::vector<size_t >> GenerateNormalHistogram( const PointCloud &point_cloud, uint8_t number_bins){
  std::vector<std::vector<size_t >> res(number_bins, std::vector<size_t>());

  for(size_t index = 0; index < point_cloud.size(); ++index){
    auto point = point_cloud[index];

    // calc normal orientation
    auto angle = normalOrientation(point, point_cloud );
    size_t bin_index = std::floor(angle / number_bins);
    res[bin_index].push_back(index);

  }

  return res;
}

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
