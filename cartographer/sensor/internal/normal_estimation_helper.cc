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

#include "cartographer/sensor/internal/normal_estimation_helper.h"

#include <cmath>
#include <vector>
#include <set>
#include <random>
#include "Eigen/Core"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

Eigen::Vector2f Pca(const std::set<size_t>& neighbors, const PointCloud& point_cloud) {
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

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(cov);

  Eigen::Vector2f eigen_vector = eigen_solver.eigenvectors().col(0);


  return eigen_vector;
}

std::set<size_t> SelectNeighborhood(const RangefinderPoint& point, const PointCloud& point_cloud){
  std::set<size_t> neighbors;
  size_t min_number_points = 3; // TODO make configurable
  float ball_radius = 0.25; // TODO make configurable

  while(neighbors.size() < min_number_points){
    for(size_t index = 0; index < point_cloud.size(); ++index){
      auto singlePoint = point_cloud[index];
      auto distance = (point.position - singlePoint.position).norm();
      if(distance <= ball_radius)
        neighbors.insert(index);
    }
    ball_radius = ball_radius + 0.5 * ball_radius;
  }

  return neighbors;
}

Eigen::Vector2f EstimateNormal(const RangefinderPoint& point, const PointCloud& point_cloud){
  //std::cerr << "SelectNeighborhood" << std::endl ;
  auto neighbors = SelectNeighborhood(point, point_cloud);
  //std::cerr << "start pca" << std::endl ;
  auto normal_candidate = Pca(neighbors, point_cloud);
  //std::cerr << "estimated normal" << std::endl ;
  normal_candidate.normalize();
  float dir = normal_candidate.dot(-point.position.head<2>()); // origin = 0
  return dir > 0 ? normal_candidate : -normal_candidate;
}

float NormalOrientation( const Eigen::Vector2f& normal) {
  float x = normal[0];
  float y = normal[1];

  float angle = 0;

  if(x > 0 && y >= 0)
    angle = std::atan(y/x);
  else if(x > 0 && y < 0)
    angle = std::atan(y/x) + 2.*M_PI;
  else if(x < 0)
    angle = std::atan(y/x) + M_PI;
  else if(x==0 && y > 0)
    angle = M_PI/2.;
  else
    angle = 3./2. * M_PI;

  return angle;
}

std::vector<std::vector<size_t >> GenerateNormalHistogram( const PointCloud &point_cloud, size_t number_bins){
  std::vector<std::vector<size_t >> res(number_bins, std::vector<size_t>());

  float slice = 2*M_PI / number_bins;

  for(size_t index = 0; index < point_cloud.size(); ++index){
    auto point = point_cloud[index];

    // calc normal orientation
    auto normal = EstimateNormal(point, point_cloud);
    auto angle = NormalOrientation( normal );
    size_t bin_index = std::floor(angle / slice);
    res[bin_index % number_bins].push_back(index);

  }

  return res;
}


}  // namespace sensor
}  // namespace cartographer
