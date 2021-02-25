//
// Created by ubuntu on 20.02.21.
//
#include <cmath>

#include "dynamic_object_removal_points_processor.h"

namespace cartographer {
namespace io {

std::unique_ptr<DynamicObjectsRemovalPointsProcessor> DynamicObjectsRemovalPointsProcessor::FromDictionary(
    common::LuaParameterDictionary *dictionary,
    PointsProcessor *next) {
  return absl::make_unique<DynamicObjectsRemovalPointsProcessor>(next);
}

DynamicObjectsRemovalPointsProcessor::DynamicObjectsRemovalPointsProcessor(PointsProcessor* next)
    : next_(next) {
  //TODO(bastian.hirschel) assign those values by passed parameters
  r_segments_ = 50.0f;
  theta_segments_ = 100.0f;
  phi_segments_ = 100.0f;
  sensor_range_limit_ = 100.0f;
}

Eigen::Vector3f DynamicObjectsRemovalPointsProcessor::cartesian_to_polar(Eigen::Vector3f cart_coord) {
  Eigen::Vector3f polar_coord;

  polar_coord.x() = sqrt(pow(cart_coord.x(), 2.0f) +
      pow(cart_coord.y(), 2.0f) + pow(cart_coord.z(), 2.0f)); // represents r
  polar_coord.y() = acos(cart_coord.z() / polar_coord.x()); // represents theta betwwen 0 and pi
  polar_coord.z() = atan2(cart_coord.y(), cart_coord.x()) + M_PI; // represents phi between 0 and +2pi
  //TODO(bastian.hirschel) check if its necessary to have it between 0 and 2pi

  return polar_coord;
}

void DynamicObjectsRemovalPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  // Create wedge for global map and current scan. Only if this isn't the first scan
  if (!map_.empty()) {
    wedge_map_t scan_wedge_map = create_wedge_map(batch->points);
    wedge_map_t global_wedge_map = create_wedge_map(sensor::TransformPointCloud(map_, batch->sensor_to_map.inverse()));
  }

  // Add the current batch to the list of batches for later sending
  list_of_batches_.push_back(*batch);

  // Add all points from the current scan to the full map
  for (auto & point : batch->points) {
    map_.push_back(batch->sensor_to_map * point);
  }

  // TODO(bastian.hirschel) we want to process upon completion of the bag
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult DynamicObjectsRemovalPointsProcessor::Flush() {
  return next_->Flush();
}

uint16_t DynamicObjectsRemovalPointsProcessor::cantor_pairing(uint16_t a, uint16_t b) {
  return ((a + b)*(a + b + 1))/2 + b;
}
DynamicObjectsRemovalPointsProcessor::wedge_key_t DynamicObjectsRemovalPointsProcessor::get_interval_segment(
    Eigen::Vector3f p) {
  float r, theta, phi;
  uint16_t r_seg, theta_seg, phi_seg;

  r = p.x();
  theta = static_cast<float>(fmod(p.y(), M_PI));
  phi = static_cast<float>(fmod(p.z(), 2.0f * M_PI));

  // Convert into positive space from 0 to pi or 0 to 2pi, respectively
  theta = theta < 0 ? theta + M_PI : theta;
  phi = phi < 0 ? phi + 2.0 * M_PI : phi;

  // Get index of the interval segment it belongs to
  r_seg = static_cast<uint16_t>(floor(r / (sensor_range_limit_ / r_segments_)));
  theta_seg = static_cast<uint16_t>(floor(theta / (M_PI / theta_segments_)));
  phi_seg = static_cast<uint16_t>(floor(phi / ((2.0 * M_PI) / phi_segments_)));

  return std::make_tuple(r_seg, theta_seg, phi_seg);
}

DynamicObjectsRemovalPointsProcessor::wedge_map_t DynamicObjectsRemovalPointsProcessor::create_wedge_map(
    sensor::PointCloud cloud) {
  wedge_map_t wedge_map;

  for (auto & p : cloud) {
    Eigen::Vector3f polar = cartesian_to_polar(p.position);
    wedge_key_t key = get_interval_segment(polar);
    auto search = wedge_map.find(key);
    if (search == wedge_map.end()) {
      // No such element existed before
      sphercial_wedge wedge;
      wedge.wedge_points.push_back(p);
      search->second = wedge;
    } else {
      // This wedge already exists, add point to the pointcloud
      search->second.wedge_points.push_back(p);
    }
  }

  return wedge_map;
}
}
}
