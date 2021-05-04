//
// Created by ubuntu on 20.02.21.
//
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <vector>

#include "dynamic_object_removal_points_processor.h"


namespace cartographer {
namespace io {
DynamicObjectsRemovalPointsProcessor::RunState DynamicObjectsRemovalPointsProcessor::run_state_ = DynamicObjectsRemovalPointsProcessor::RunState::kInitialRun;

void WriteBinaryPlyHeader(const bool has_color, const bool has_intensities,
                          const std::vector<std::string>& comments,
                          const int64 num_points,
                          FileWriter* const file_writer) {
  const std::string color_header = !has_color ? ""
                                              : "property uchar red\n"
                                                "property uchar green\n"
                                                "property uchar blue\n";
  const std::string intensity_header =
      !has_intensities ? "" : "property float intensity\n";
  std::ostringstream stream;
  stream << "ply\n"
         << "format binary_little_endian 1.0\n"
         << "comment generated by Cartographer\n";
  for (const std::string& comment : comments) {
    stream << "comment " << comment << "\n";
  }
  stream << "element vertex " << std::setw(15) << std::setfill('0')
         << num_points << "\n"
         << "property float x\n"
         << "property float y\n"
         << "property float z\n"
         << color_header << intensity_header << "end_header\n";
  const std::string out = stream.str();
  CHECK(file_writer->WriteHeader(out.data(), out.size()));
}

void WriteBinaryPlyPointCoordinate(const Eigen::Vector3f& point,
                                   FileWriter* const file_writer) {
  // TODO(sirver): This ignores endianness.
  char buffer[12];
  memcpy(buffer, &point[0], sizeof(float));
  memcpy(buffer + 4, &point[1], sizeof(float));
  memcpy(buffer + 8, &point[2], sizeof(float));
  CHECK(file_writer->Write(buffer, 12));
}

std::unique_ptr<DynamicObjectsRemovalPointsProcessor> DynamicObjectsRemovalPointsProcessor::FromDictionary(
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary *dictionary,
    PointsProcessor *next) {
  return absl::make_unique<DynamicObjectsRemovalPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")),
      dictionary->GetInt("r_segments"),
      dictionary->GetInt("theta_segments"),
      dictionary->GetInt("phi_segments"),
      dictionary->GetDouble("sensor_range_limit"),
      dictionary->GetInt("end_of_file"),
      next);
}

DynamicObjectsRemovalPointsProcessor::DynamicObjectsRemovalPointsProcessor(std::unique_ptr<FileWriter> file_writer,
    const int r_segments, const int theta_segments, const int phi_segments, const float sensor_range_limit,
    const int end_of_file, PointsProcessor* next)
    : r_segments_(r_segments), theta_segments_(theta_segments),
    phi_segments_(phi_segments), end_of_file_(end_of_file), sensor_range_limit_(sensor_range_limit),
    next_(next), file_(std::move(file_writer)) {
  LOG(INFO) << "Initialized Dynamic objects removal filter with:\n" <<
  "r_segments:         " << r_segments_ << "\n" <<
  "theta_segments:     " << theta_segments_ << "\n" <<
  "phi_segments:       " << phi_segments_ << "\n" <<
  "sensor_range_limit: " << sensor_range_limit_ << "\n" <<
  "end_of_file:        " << end_of_file_;
  sensor_height_adjustment_ = transform::Rigid3<float>::Translation(Eigen::Vector3f(0, 0, -0.5));
}

void DynamicObjectsRemovalPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  // For debugging: sleep at first iteration
//  if (map_.empty()) {
//    std::this_thread::sleep_for(std::chrono::seconds(10));
//  }

  switch (run_state_) {
    case RunState::kInitialRun:
      LOG(INFO) << "Iteration: " << list_of_batches_.size() + 1 << "\tBatch points: " << batch->points.size();
      LOG(INFO) << "Batch origin:      x: " << batch->origin.x() << "\ty: " << batch->origin.y() << "\tz: " << batch->origin.z();
      LOG(INFO) << "Batch transformation: " << batch->sensor_to_map.DebugString();

//      if (list_of_batches_.size() == end_of_file_-1) {
//        std::vector<std::string> comments;
//        WriteBinaryPlyHeader(false, false, comments, 11760, file_.get());
//        for (size_t i = 0; i < batch->points.size(); ++i) {
//          WriteBinaryPlyPointCoordinate(batch->sensor_to_map * batch->points[i].position,
//                                        file_.get());
//        }
//        WriteBinaryPlyHeader(false, false, comments, batch->points.size(),
//                             file_.get());
//      }

      // Create wedge for global map and current scan. Only if this isn't the first sca
      if (!map_.empty()) {
        wedge_map_t scan_wedge_map = create_wedge_map(sensor::TransformPointCloud(batch->points, sensor_height_adjustment_));
        wedge_map_t global_wedge_map = create_wedge_map(/*sensor::TransformPointCloud(*/map_/*, batch->sensor_to_map.inverse())*/);

        LOG(INFO) << "Scan wedge map size: " << scan_wedge_map.size() << "\tGlobal wedge map size: " << global_wedge_map.size();

        // Test writing of points
        if (list_of_batches_.size() == end_of_file_-1) {
          std::srand(std::time(nullptr));
          sphercial_wedge wedge;
          float r_step = sensor_range_limit_ / r_segments_;
          float r_min = 0.0f * r_step;
          float r_max = 15.0f;//r_min + r_step;
          float theta_step = M_PIf / theta_segments_;
          float theta_min = 15.0f * theta_step;
          float theta_max = theta_min + theta_step;
          float phi_step = (2.0f * M_PIf) / phi_segments_;
          float phi_min = 0.0f * phi_step;
          float phi_max = phi_min + phi_step;

          for (int i = 0; i < 1000; ++i) {
            float r = r_min + static_cast<float>(rand()) / (static_cast <float> (RAND_MAX/(r_max-r_min)));
            float theta = theta_min + static_cast<float>(rand()) / (static_cast <float> (RAND_MAX/(theta_max-theta_min)));
            float phi = phi_min + static_cast<float>(rand()) / (static_cast <float> (RAND_MAX/(phi_max-phi_min)));
            sensor::RangefinderPoint coords;
            coords.position = polar_to_cartesian(r, theta, phi);
            wedge.wedge_points.push_back(coords);
          }

          std::vector<std::string> comments;

          WriteBinaryPlyHeader(false, false, comments, 0, file_.get());
          for (size_t i = 0; i < batch->points.size(); ++i) {
            WriteBinaryPlyPointCoordinate(batch->sensor_to_map * batch->points[i].position, file_.get());
          }
          WriteBinaryPlyHeader(false, false, comments, batch->points.size(),
                               file_.get());
          /*for (size_t i = 0; i < wedge.wedge_points.size(); ++i) {
            WriteBinaryPlyPointCoordinate(wedge.wedge_points[i].position, file_.get());
          }
          WriteBinaryPlyHeader(false, false, comments, 75*1000,
                               file_.get());*/
          /*for (size_t i = 0; i < batch->points.size(); ++i) {
            WriteBinaryPlyPointCoordinate(batch->points[i].position, file_.get());
          }
          WriteBinaryPlyHeader(false, false, comments, batch->points.size(),
                               file_.get());*/
          /*for (size_t i = 0; i < map_.size(); ++i) {
            WriteBinaryPlyPointCoordinate(map_[i].position, file_.get());
          }
          WriteBinaryPlyHeader(false, false, comments, map_.size(),
                               file_.get());*/
          /*size_t num_to_write = 0;
          for (int new_r = 0; new_r <= r_segments_; ++new_r) {
            wedge_key_t new_key = std::make_tuple(new_r, 12, 0);

            auto search = global_wedge_map.find(new_key);
            if (search != global_wedge_map.end()) {
              // Wedge exists
              num_to_write += search->second.wedge_points.size();
              for (int i = 0; i < search->second.wedge_points.size(); ++i) {
                WriteBinaryPlyPointCoordinate(search->second.wedge_points[i].position, file_.get());
              }
            }
          }
          WriteBinaryPlyHeader(false, false, comments, num_to_write,
                               file_.get());*/
          CHECK(file_->Close()) << "Closing PLY file_writer failed.";
        }

        // Dynamic objects detection
        size_t total_number_removed_points = 0;
        std::vector<wedge_key_t> keys_to_delete;
        size_t total_to_be_removed, already_removed;

        total_to_be_removed = 0;
        already_removed = 0;

        for (uint16_t new_theta = 0; new_theta < theta_segments_; ++new_theta) {
          for (uint16_t new_phi = 0; new_phi < phi_segments_; ++new_phi) {
            // Iterate over all scan wedges with same theta and phi but increasing distance r starting
            // at 0 to determine wedge with greatest cardinality
            std::pair<int, int> max_cardinality = std::make_pair(-1, -1);

            for (uint16_t new_r = 0; new_r < r_segments_; ++new_r) {
              wedge_key_t new_key = std::make_tuple(new_r, new_theta, new_phi);

              auto search = scan_wedge_map.find(new_key);
              if (search != scan_wedge_map.end()) {
                // Wedge exists
                // Cardinality check: find wedge with greatest cardinality
                if (max_cardinality.first < 0 || max_cardinality.second < 0 ||
                    max_cardinality.second < search->second.wedge_points.size()) {
                  max_cardinality = std::make_pair(new_r, search->second.wedge_points.size());
                }
              }
            }
            if (max_cardinality.first >= 0 && max_cardinality.second >= 0) {
              // TODO(bhirschel) maybe set a minimum number of detections to make it significant
              // Significant detection perceived. Check if dynamic object
    //          LOG(INFO) << "Got " << max_cardinality.second << " detections at (" << max_cardinality.first << ", " << new_theta << ", " << new_phi << ")";
              uint16_t r_scan_detection = max_cardinality.first - 1; // TODO(bhirschel) try adding a little tolerance here

              for (uint16_t r_to_delete = 0; r_to_delete < r_scan_detection; ++r_to_delete) {
                wedge_key_t new_key = std::make_tuple(r_to_delete, new_theta, new_phi);
                keys_to_delete.push_back(new_key);
                total_to_be_removed += global_wedge_map[new_key].wedge_points.size();

                // Remove from the global wedge map. Necessary for potential future iterations over this ray in this time segment
                global_wedge_map.erase(new_key);
              }
    //          LOG(INFO) << total_to_be_removed << " to be removed from the map";
            }
          }
        }
        // Remove the points from the global map and from all local batches
        // Remove from the global map
        remove_points_from_pointcloud(keys_to_delete, map_, batch->sensor_to_map.inverse());
        // Remove from the list of batches
        // iterate the list of batches revere because it is observable that
        // the last batch often is the main contributor of an dynamic objects' remaining
        // spurious trail. Count the number of points from the global map and break the loop if
        // this number was removed in total for all the individual batches
        for (auto batch_iter = list_of_batches_.rbegin(); batch_iter != list_of_batches_.rend(); ++batch_iter) {
          //transform::Rigid3<float> transformation = batch->sensor_to_map.inverse() * batch_iter->sensor_to_map;  // TODO(bhirschel) verify order of linked transformations
          transform::Rigid3<float> transformation = sensor_height_adjustment_; // Currently only testing sensor height adjustment
              //            LOG(INFO) << "Size before: " << batch_iter.points.size();
          already_removed += remove_points_from_batch(keys_to_delete, *batch_iter, transformation);
    //            LOG(INFO) << "Size after: " << batch_iter.points.size();

          if (already_removed >= total_to_be_removed) {
            //LOG(INFO) << "Removal cancelled early";
            break;
          }
        }
        total_number_removed_points += already_removed;
        LOG(INFO) << "Total number of removed points: " << total_number_removed_points;
      }

      // Add the current batch to the list of batches for later sending
      list_of_batches_.push_back(*batch);
      LOG(INFO) << "Batch inserted";

      // Add all points from the current scan to the full map
      for (auto & point : batch->points) {
        map_.push_back(/*batch->sensor_to_map * */ sensor_height_adjustment_ * point);
      }

      LOG(INFO) << "Total Map points: " << map_.size();
      break;
    case RunState::kSecondRun:
      //LOG(INFO) << "Start sending batches through pipeline! No.: " << list_of_batches_.size();
      auto this_batch = list_of_batches_[0];
      LOG(INFO) << "Batch sent. Size: " << this_batch.points.size();
      next_->Process(std::make_unique<PointsBatch>(this_batch));
      list_of_batches_.erase(list_of_batches_.begin());
  }

  //next_->Process(std::move(batch));
}

PointsProcessor::FlushResult DynamicObjectsRemovalPointsProcessor::Flush() {
  LOG(INFO) << "Flushing dynamic_object_removal_points_processor";
  switch (run_state_) {
    case RunState::kInitialRun:
      run_state_ = RunState::kSecondRun;
      return FlushResult::kRestartStream;
    default:
      return next_->Flush();
  }
}

Eigen::Vector3f DynamicObjectsRemovalPointsProcessor::cartesian_to_polar(Eigen::Vector3f cart_coord) {
  Eigen::Vector3f polar_coord;

  polar_coord.x() = cart_coord.norm(); // represents r
  polar_coord.y() = acosf((cart_coord.z() / polar_coord.x())); // represents theta between 0 and pi
  polar_coord.z() = atan2f(cart_coord.y(), cart_coord.x()) + M_PIf; // represents phi between 0 and +2pi

  return polar_coord;
}

Eigen::Vector3f DynamicObjectsRemovalPointsProcessor::polar_to_cartesian(float r,
                                                                         float theta,
                                                                         float phi) {
  Eigen::Vector3f cart;

  cart.x() = r*sinf(theta)*cosf(phi - M_PIf);
  cart.y() = r*sinf(theta)*sinf(phi - M_PIf);
  cart.z() = r*cosf(theta);

  return cart;
}

uint16_t DynamicObjectsRemovalPointsProcessor::cantor_pairing(uint16_t a, uint16_t b) {
  return ((a + b)*(a + b + 1))/2 + b;
}

DynamicObjectsRemovalPointsProcessor::wedge_key_t DynamicObjectsRemovalPointsProcessor::get_interval_segment(
    Eigen::Vector3f p) {
  float r, theta, phi;
  uint16_t r_seg, theta_seg, phi_seg;

  r = p.x();
  theta = static_cast<float>(fmod(p.y(), M_PIf));
  phi = static_cast<float>(fmod(p.z(), 2.0f * M_PIf));

  // Convert into positive space from 0 to pi or 0 to 2pi, respectively
  theta = theta < 0 ? theta + M_PIf : theta;
  phi = phi < 0 ? phi + 2.0f * M_PIf : phi;

  // Get index of the interval segment it belongs to
  r_seg = static_cast<uint16_t>(floorf(r / (sensor_range_limit_ / r_segments_)));
  theta_seg = static_cast<uint16_t>(floorf(theta / (M_PIf / theta_segments_)));
  phi_seg = static_cast<uint16_t>(floorf(phi / ((2.0f * M_PIf) / phi_segments_)));

  return std::make_tuple(r_seg, theta_seg, phi_seg);
}

DynamicObjectsRemovalPointsProcessor::wedge_map_t DynamicObjectsRemovalPointsProcessor::create_wedge_map(
    sensor::PointCloud cloud) {
  wedge_map_t wedge_map;

  for (auto & p : cloud) {
    Eigen::Vector3f polar = cartesian_to_polar(p.position);

    // Skip if distance is greater than the sensor range limit
    if (polar.x() > sensor_range_limit_) {
      continue;
    }

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

    // Test of cartesian to polar and vice versa
    /*Eigen::Vector3f rev = polar_to_cartesian(polar.x(), polar.y(), polar.z());
    LOG(INFO) << "ORG: x: " << p.position.x() << "\ty: " << p.position.y() << "\tz: " << p.position.z();
    LOG(INFO) << "POL: r: " << polar.x() << "\tt: " << polar.y() << "\tp: " << polar.z();
    LOG(INFO) << "REV: x: " << rev.x() << "\ty: " << rev.y() << "\tz: " << rev.z();*/

    // Test of key
//    LOG(INFO) << "ORG: x: " << p.position.x() << "\ty: " << p.position.y() << "\tz: " << p.position.z();
//    LOG(INFO) << "POL: r: " << polar.x() << "\tt: " << polar.y() << "\tp: " << polar.z();
//    LOG(INFO) << "KEY: r: " << std::get<0>(key) <<"\tt: " << std::get<1>(key) << "\tp: " << std::get<2>(key);
  }

  return wedge_map;
}

void DynamicObjectsRemovalPointsProcessor::remove_points_from_pointcloud(std::vector<wedge_key_t> keys_to_delete,
                                                                         sensor::PointCloud &cloud,
                                                                         transform::Rigid3<float> transformation) {
  absl::flat_hash_set<int> to_remove;

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto p = /*transformation * */cloud[i].position;
    wedge_key_t local_key = get_interval_segment(cartesian_to_polar(p));
    if (std::find(keys_to_delete.begin(), keys_to_delete.end(), local_key) != keys_to_delete.end()) {
      to_remove.insert(i);
    }
  }

  const size_t new_num_points = cloud.size() - to_remove.size();
  sensor::PointCloud points;
  points.reserve(new_num_points);
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (to_remove.count(i) == 1) {
      continue;
    }
    points.push_back(cloud[i]);
  }
  cloud = std::move(points);

//  LOG(INFO) << "Removed from global map: " << to_remove.size() << " points";
}
size_t DynamicObjectsRemovalPointsProcessor::remove_points_from_batch(std::vector<wedge_key_t> keys_to_delete,
                                                                    PointsBatch &batch,
                                                                    transform::Rigid3<float> transformation) {
  absl::flat_hash_set<int> to_remove;
  size_t batch_size = batch.points.size();

  for (size_t i = 0; i < batch.points.size(); ++i) {
    wedge_key_t local_key = get_interval_segment(cartesian_to_polar(transformation * batch.points[i].position));
    if (std::find(keys_to_delete.begin(), keys_to_delete.end(), local_key) != keys_to_delete.end()) {
      to_remove.insert(i);
    }
  }

  if (!to_remove.empty()) {
    RemovePoints(to_remove, &batch);
//    LOG(INFO) << "Local batch target removal: " << to_remove.size() << " points, " <<
//      batch_size - batch.points.size() << " points removed";
  }
  return to_remove.size();
}
}
}
