//
// Created by bhirschel on 20.02.21.
//
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <vector>

#include "dynamic_objects_removal_points_processor.h"
#include "cartographer/io/color.h"

namespace cartographer {
namespace io {
DynamicObjectsRemovalPointsProcessor::RunState
    DynamicObjectsRemovalPointsProcessor::run_state_ =
    DynamicObjectsRemovalPointsProcessor::RunState::kInitialRun;

std::unique_ptr<DynamicObjectsRemovalPointsProcessor> DynamicObjectsRemovalPointsProcessor::FromDictionary(
    common::LuaParameterDictionary *dictionary,
    PointsProcessor *next) {
  return absl::make_unique<DynamicObjectsRemovalPointsProcessor>(
      dictionary->GetInt("r_segments"),
      dictionary->GetInt("theta_segments"),
      dictionary->GetInt("phi_segments"),
      dictionary->GetDouble("sensor_range_limit"),
      dictionary->GetDouble("probability_reduction_factor"),
      dictionary->GetDouble("dynamic_object_probability_threshold"),
      dictionary->GetDouble("search_ray_threshold"),
      dictionary->HasKey("open_view_deletion") ? dictionary->GetBool(
          "open_view_deletion") : false,
      dictionary->HasKey("show_extended_debug_information")
      ? dictionary->GetBool("show_extended_debug_information") : false,
      next);
}

DynamicObjectsRemovalPointsProcessor::DynamicObjectsRemovalPointsProcessor(
    const int r_segments,
    const int theta_segments,
    const int phi_segments,
    const double sensor_range_limit,
    const double probability_reduction_factor,
    const double dynamic_object_probability_threshold,
    const double search_ray_threshold,
    const bool open_view_deletion,
    const bool show_extended_debug_information,
    PointsProcessor *next)
    : r_segments_(r_segments),
      theta_segments_(theta_segments),
      phi_segments_(phi_segments),
      sensor_range_limit_(sensor_range_limit),
      probability_reduction_factor_(probability_reduction_factor),
      dynamic_object_probability_threshold_(dynamic_object_probability_threshold),
      search_ray_threshold_(search_ray_threshold),
      open_view_deletion_(open_view_deletion),
      show_extended_debug_information_(show_extended_debug_information),
      next_(next) {
  if (show_extended_debug_information_) {
    LOG(INFO) << "Initialized Dynamic objects removal filter with:\n" <<
              "r_segments:                            " << r_segments_ << "\n"
              <<
              "theta_segments:                        " << theta_segments_
              << "\n" <<
              "phi_segments:                          " << phi_segments_ << "\n"
              <<
              "sensor_range_limit:                    " << sensor_range_limit_
              << "\n" <<
              "probability_reduction_factor:          "
              << probability_reduction_factor_ << "\n" <<
              "dynamic_object_probability_threshold:  "
              << dynamic_object_probability_threshold_ << "\n" <<
              "search_ray_threshold:                  " << search_ray_threshold_
              << "\n" <<
              "open_view_deletion:                    "
              << (open_view_deletion_ ? "True" : "False");
  }

  // Initialize max range for scan batch
  scan_batch_max_range_ = static_cast<uint16_t>(r_segments_);
  eval_total_points_ = 0;
  eval_total_time_begin_ = std::chrono::high_resolution_clock::now();
}

void DynamicObjectsRemovalPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  ++iteration_;

  if (run_state_ == RunState::kInitialRun) {
    auto begin = std::chrono::high_resolution_clock::now();

    robot_translation_ =
        transform::Rigid3<float>(batch->sensor_to_map.translation(),
                                 Eigen::Quaternion<float>::Identity()).inverse();

    // Create scan wedges
    sensor::CustomPointCloud scan_map;
    initialize_scan_map(scan_map, batch.get(), iteration_);
    wedge_map_t scan_wedge_map =
        create_wedge_map(sensor::TransformCustomPointCloud(scan_map,
                                                           robot_translation_),
                         true);

    if (!map_.empty()) {
      // Create global wedges
      wedge_map_t global_wedge_map =
          create_wedge_map(sensor::TransformCustomPointCloud(map_,
                                                             robot_translation_),
                           false);

      // Dynamic objects detection
      size_t total_number_removed_points = 0;

      // Detection of points of type 2 (delete map points in front of current
      // detection points)
      std::unordered_map<std::pair<uint16_t, uint16_t>,
                         int,
                         key_hash_pair,
                         key_equal_pair>
          scan_wedge_map_cardinalities;

      for (uint16_t theta_iter = 0; theta_iter < theta_segments_;
           ++theta_iter) {
        for (uint16_t phi_iter = 0; phi_iter < phi_segments_; ++phi_iter) {
          // Iterate over all scan wedges with same theta and phi but
          // increasing distance r starting at 0 to determine wedge with
          // greatest cardinality
          std::pair<int, int> max_cardinality = std::make_pair(-1, -1);

          for (uint16_t r_iter = 0; r_iter < scan_batch_max_range_;
               ++r_iter) {
            wedge_key_t
                search_key = std::make_tuple(r_iter, theta_iter, phi_iter);

            auto search = scan_wedge_map.find(search_key);
            if (search != scan_wedge_map.end()) {
              // Wedge exists
              // Cardinality check: find wedge with greatest cardinality
              if (max_cardinality.first < 0 || max_cardinality.second < 0 ||
                  1.2f * max_cardinality.second
                      < static_cast<float>(search->second.wedge_points.size())) {
                max_cardinality = std::make_pair(r_iter,
                                                 search->second.wedge_points.size());
              }
            }
          }

          scan_wedge_map_cardinalities[std::make_pair(theta_iter, phi_iter)] =
              max_cardinality.first;

          if (max_cardinality.first >= 0
              && max_cardinality.first <= r_segments_
              && max_cardinality.second >= 0) {
            // Significant detection perceived. Check if dynamic object
            uint16_t r_scan_detection = max_cardinality.first;

            for (uint16_t r_to_lower = 0;
                 r_to_lower < search_ray_threshold_ * r_scan_detection;
                 ++r_to_lower) {
              wedge_key_t key_to_lower =
                  std::make_tuple(r_to_lower, theta_iter, phi_iter);

              // We lower the probability of all points in this wedge
              for (auto &p : global_wedge_map[key_to_lower].wedge_points) {
                p.probability -=
                    static_cast<float>(probability_reduction_factor_);
              }
            }
          }
        }
      }

      //Detection of points of type 3 (delete map points with empty scan ray)
      if (open_view_deletion_) {
        for (auto &wedge : global_wedge_map) {
          if (std::get<0>(wedge.first) > r_segments_) {
            // Skip points that are too far away
            // (outside maximal number of range segments)
            continue;
          }

          wedge_key_t this_key;
          int scan_wedge_cardinality;
          this_key = wedge.first;
          scan_wedge_cardinality =
              scan_wedge_map_cardinalities[std::make_pair(std::get<1>(this_key),
                                                          std::get<2>(this_key))];

          if (scan_wedge_cardinality == -1) {
            // Scan wedge doesn't have any significant detections for this
            // direction of theta & phi. Lower the probability of all points
            // in this wedge
            for (auto &p : wedge.second.wedge_points) {
              p.probability -=
                  static_cast<float>(probability_reduction_factor_);
            }
          }
        }
      }

      // Add those points to the global map from the global wedge map, which
      // have a probability high enough. Apply backwards transformation since
      // wedge map points are in the robot frame
      map_.clear();
      for (auto &wedge : global_wedge_map) {
        for (auto &p : wedge.second.wedge_points) {
          if (p.probability
              >= static_cast<float>(dynamic_object_probability_threshold_)) {
            // Apply backwards transformation back to map frame
            map_.push_back(robot_translation_.inverse() * p);
          } else {
            // Point was not taken over since its probability is too low.
            // Will be removed from the map and batches
            total_number_removed_points++;
          }
        }
      }
      eval_total_points_ += total_number_removed_points;
      eval_number_deleted_points_.push_back(total_number_removed_points);
    }

    // Add all points from the current scan to the full map. Apply backwards
    // transformation since wedge map points are in the robot frame
    for (auto &wedge : scan_wedge_map) {
      for (auto &point : wedge.second.wedge_points) {
        map_.push_back(robot_translation_.inverse() * point);
      }
    }

    // Add the current batch to the list of batches for later sending and
    // clear fields that are saved in the map_
    batch->points.clear();
    batch->intensities.clear();
    batch->colors.clear();
    list_of_batches_.push_back(*batch);

    auto end = std::chrono::high_resolution_clock::now();
    eval_time_detailed_.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
        end - begin));
    if (show_extended_debug_information_) {
      LOG(INFO) << "[DORPP - Iteration " << iteration_ << "]: "
                << "Total n.o. points: " << map_.size()
                << ", time elapsed: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - begin).count() << " ms";
    }

    eval_cumulated_number_of_points_.push_back(map_.size());

  }else {
    // At the second run, send the batches through the pipeline
    auto this_batch = list_of_batches_[iteration_-1];
    next_->Process(std::make_unique<PointsBatch>(this_batch));
//    list_of_batches_.erase(list_of_batches_.begin());
  }
}

PointsProcessor::FlushResult DynamicObjectsRemovalPointsProcessor::Flush() {
  if (run_state_ == RunState::kInitialRun) {
    flush_points_to_batch();
    run_state_ = RunState::kSecondRun;

    auto end = std::chrono::high_resolution_clock::now();
    eval_total_time_elapsed_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end - eval_total_time_begin_);
    iteration_ = 0;

    return FlushResult::kRestartStream;
  } else {
    iteration_ = 0;
    if (show_extended_debug_information_) {
      LOG(INFO) << "Total time: " << eval_total_time_elapsed_.count() << " ms";
      LOG(INFO) << "Total number removed points: " << eval_total_points_
                << " from " << map_.size() << " (ratio: "
                << (static_cast<double>(eval_total_points_)
                    / (eval_total_points_ + map_.size())) << ")";
      std::ostringstream time_out, cum_points_out, del_points_out;
      for (auto &time : eval_time_detailed_) {
        time_out << time.count() << ";";
      }
      LOG(INFO) << "Detailed timing: " << time_out.str();

      for (auto &cum_points : eval_cumulated_number_of_points_) {
        cum_points_out << cum_points << ";";
      }
      LOG(INFO) << "Detailed cumulated n.o. points: " << cum_points_out.str();

      for (auto &del_points : eval_number_deleted_points_) {
        del_points_out << del_points << ";";
      }
      LOG(INFO) << "Detailed n.o. deleted points: " << del_points_out.str();
    }

    return next_->Flush();
  }
}

Eigen::Vector3f DynamicObjectsRemovalPointsProcessor::cartesian_to_polar(Eigen::Vector3f cart_coord) {
  Eigen::Vector3f polar_coord;

  polar_coord.x() = cart_coord.norm(); // represents r
  polar_coord.y() = acosf((cart_coord.z()
      / polar_coord.x())); // represents theta between 0 and pi
  polar_coord.z() =
      atan2f(cart_coord.y(), cart_coord.x())
          + M_PIf; // represents phi between 0 and +2pi

  return polar_coord;
}

Eigen::Vector3f DynamicObjectsRemovalPointsProcessor::polar_to_cartesian(float r,
                                                                         float theta,
                                                                         float phi) {
  Eigen::Vector3f cart;

  cart.x() = r * sinf(theta) * cosf(phi - M_PIf);
  cart.y() = r * sinf(theta) * sinf(phi - M_PIf);
  cart.z() = r * cosf(theta);

  return cart;
}

uint16_t DynamicObjectsRemovalPointsProcessor::cantor_pairing(uint16_t a,
                                                              uint16_t b) {
  return ((a + b) * (a + b + 1)) / 2 + b;
}

DynamicObjectsRemovalPointsProcessor::wedge_key_t DynamicObjectsRemovalPointsProcessor::get_interval_segment(
    Eigen::Vector3f p) const {
  float r, theta, phi;
  uint16_t r_seg, theta_seg, phi_seg;

  r = p.x();
  theta = static_cast<float>(fmod(p.y(), M_PIf));
  phi = static_cast<float>(fmod(p.z(), 2.0f * M_PIf));

  // Convert into positive space from 0 to pi or 0 to 2pi, respectively
  theta = theta < 0 ? theta + M_PIf : theta;
  phi = phi < 0 ? phi + 2.0f * M_PIf : phi;

  // Get index of the interval segment it belongs to
  r_seg =
      static_cast<uint16_t>(floorf(r / (sensor_range_limit_ / r_segments_)));
  theta_seg = static_cast<uint16_t>(floorf(theta / (M_PIf / theta_segments_)));
  phi_seg =
      static_cast<uint16_t>(floorf(phi / ((2.0f * M_PIf) / phi_segments_)));

  return std::make_tuple(r_seg, theta_seg, phi_seg);
}

DynamicObjectsRemovalPointsProcessor::wedge_map_t DynamicObjectsRemovalPointsProcessor::create_wedge_map(
    const sensor::CustomPointCloud &cloud, bool is_scan_batch) {
  wedge_map_t wedge_map;

  for (auto &p : cloud) {
    Eigen::Vector3f polar = cartesian_to_polar(p.position);

    wedge_key_t key = get_interval_segment(polar);
    auto search = wedge_map.find(key);
    if (search == wedge_map.end()) {
      // No such element existed before
      sphercial_wedge wedge;
      wedge.wedge_points.push_back(p);
      wedge_map[key] = wedge;
    } else {
      // This wedge already exists, add point to the pointcloud
      search->second.wedge_points.push_back(p);
    }

    if (is_scan_batch && std::get<0>(key) > scan_batch_max_range_) {
      scan_batch_max_range_ = std::get<0>(key);
    }
  }

  return wedge_map;
}

void DynamicObjectsRemovalPointsProcessor::initialize_scan_map(sensor::CustomPointCloud &scan_map,
                                                               PointsBatch *batch,
                                                               int index) {
  FloatColor nan_color = {{NAN, NAN, NAN}};
  for (size_t i = 0; i < batch->points.size(); ++i) {
    float intensity = batch->intensities.empty() ? NAN : batch->intensities[i];
    FloatColor color = batch->colors.empty() ? nan_color : batch->colors[i];
    scan_map.push_back({batch->points[i].position, 1.0f, index, intensity,
                        color});
  }
}

void DynamicObjectsRemovalPointsProcessor::flush_points_to_batch() {
  for (auto &point : map_) {
    if (point.probability >= dynamic_object_probability_threshold_
        && static_cast<u_long>(point.index) < list_of_batches_.size()) {
      sensor::RangefinderPoint extracted_point;
      extracted_point.position = point.position;
      list_of_batches_[point.index].points.push_back(extracted_point);
      // Only set intensities if they were originally set (not nan)
      if (!std::isnan(point.intensity)) {
        list_of_batches_[point.index].intensities.push_back(point.intensity);
      }
      // Only set colors if they were originally set (not nan)
      if (!std::isnan(point.color[0])) {
        list_of_batches_[point.index].colors.push_back(point.color);
      }
    }
  }
}

}
}
