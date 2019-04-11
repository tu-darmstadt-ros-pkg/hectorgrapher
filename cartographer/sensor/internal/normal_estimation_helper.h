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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_NORMAL_ESTIMATION_HELPER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_NORMAL_ESTIMATION_HELPER_H_


#include "cartographer/sensor/point_cloud.h"



namespace cartographer {
namespace sensor {

Eigen::Vector2f Pca(const std::set<size_t>& neighbors, const PointCloud& point_cloud);

std::set<size_t> SelectNeighborhood(const RangefinderPoint& point, const PointCloud& point_cloud);

Eigen::Vector2f EstimateNormal(const RangefinderPoint& point, const PointCloud& point_cloud);

float NormalOrientation( const Eigen::Vector2f& normal);

std::vector<std::vector<size_t >> GenerateNormalHistogram( const PointCloud &point_cloud, size_t number_bins);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_NORMAL_ESTIMATION_HELPER_H_
