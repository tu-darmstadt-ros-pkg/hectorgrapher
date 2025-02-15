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

#include "cartographer/mapping/3d/submap_3d.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/sensor/range_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

struct PixelData {
  int min_z = INT_MAX;
  int max_z = INT_MIN;
  int count = 0;
  float probability_sum = 0.f;
  float max_probability = 0.5f;
};

// Filters 'range_data', retaining only the returns that have no more than
// 'max_range' distance from the origin. Removes misses and reflectivity
// information.
sensor::RangeData FilterRangeDataByMaxRange(const sensor::RangeData& range_data,
                                            const float max_range) {
  sensor::RangeData result{range_data.origin, {}, {}, range_data.width};
  if(max_range == 0.f) {
    result.returns = range_data.returns;
    return result;
  }

  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    if ((hit.position - range_data.origin).norm() <= max_range) {
      result.returns.push_back(hit);
    }
  }
  return result;
}

// Filters structured 'range_data', setting all points with a distance from the
// origin smaller than 'max_range' to the origin. Therby the cloud size and
// structure is preserved. Removes misses and reflectivity information.
sensor::RangeData FilterStructuredRangeDataByMaxRange(
    const sensor::RangeData& range_data, const float max_range) {
  sensor::RangeData result{
      range_data.origin, range_data.returns, {}, range_data.width};
  if (max_range == 0.f) {
    result.returns = range_data.returns;
    return result;
  }

  for (sensor::RangefinderPoint& hit : result.returns) {
    if ((hit.position - result.origin).norm() > max_range) {
      hit.position = result.origin;
    }
  }
  return result;
}

std::vector<PixelData> AccumulatePixelData(
    const int width, const int height, const Eigen::Array2i& min_index,
    const Eigen::Array2i& max_index,
    const std::vector<Eigen::Array4i>& voxel_indices_and_probabilities) {
  std::vector<PixelData> accumulated_pixel_data(width * height);
  for (const Eigen::Array4i& voxel_index_and_probability :
       voxel_indices_and_probabilities) {
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    if ((pixel_index < min_index).any() || (pixel_index > max_index).any()) {
      // Out of bounds. This could happen because of floating point inaccuracy.
      continue;
    }
    const int x = max_index.x() - pixel_index[0];
    const int y = max_index.y() - pixel_index[1];
    PixelData& pixel = accumulated_pixel_data[x * width + y];
    ++pixel.count;
    pixel.min_z = std::min(pixel.min_z, voxel_index_and_probability[2]);
    pixel.max_z = std::max(pixel.max_z, voxel_index_and_probability[2]);
    const float probability =
        ValueToProbability(voxel_index_and_probability[3]);
    pixel.probability_sum += probability;
    pixel.max_probability = std::max(pixel.max_probability, probability);
  }
  return accumulated_pixel_data;
}

// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
std::vector<Eigen::Array4i> ExtractVoxelData(
    const HybridGrid& hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const uint16 probability_value = it.GetValue();
    const float probability = ValueToProbability(probability_value);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);

    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  return voxel_indices_and_probabilities;
}

// The first three entries of each returned value are a cell_index and the
// last is the corresponding probability value. We batch them together like
// this to only have one vector and have better cache locality.
std::vector<Eigen::Array4i> ExtractVoxelData(
    const HybridGridTSDF& hybrid_grid, const transform::Rigid3f& transform,
    Eigen::Array2i* min_index, Eigen::Array2i* max_index) {
  std::vector<Eigen::Array4i> voxel_indices_and_probabilities;
  const float resolution_inverse = 1.f / hybrid_grid.resolution();

  constexpr float kXrayObstructedCellProbabilityLimit = 0.501f;
  for (auto it = HybridGridTSDF::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    const TSDFVoxel voxel = it.GetValue();
    const float tsd =
        hybrid_grid.ValueConverter().ValueToTSD(voxel.discrete_tsd);
    const float probability =
        1.f - std::abs(tsd) / hybrid_grid.ValueConverter().getMaxTSD();
    const float probability_value = ProbabilityToValue(probability);
    if (probability < kXrayObstructedCellProbabilityLimit) {
      // We ignore non-obstructed cells.
      continue;
    }

    const Eigen::Vector3f cell_center_submap =
        hybrid_grid.GetCenterOfCell(it.GetCellIndex());
    const Eigen::Vector3f cell_center_global = transform * cell_center_submap;
    const Eigen::Array4i voxel_index_and_probability(
        common::RoundToInt(cell_center_global.x() * resolution_inverse),
        common::RoundToInt(cell_center_global.y() * resolution_inverse),
        common::RoundToInt(cell_center_global.z() * resolution_inverse),
        probability_value);

    voxel_indices_and_probabilities.push_back(voxel_index_and_probability);
    const Eigen::Array2i pixel_index = voxel_index_and_probability.head<2>();
    *min_index = min_index->cwiseMin(pixel_index);
    *max_index = max_index->cwiseMax(pixel_index);
  }
  return voxel_indices_and_probabilities;
}

// Builds texture data containing interleaved value and alpha for the
// visualization from 'accumulated_pixel_data'.
std::string ComputePixelValues(
    const std::vector<PixelData>& accumulated_pixel_data) {
  std::string cell_data;
  cell_data.reserve(2 * accumulated_pixel_data.size());
  constexpr float kMinZDifference = 3.f;
  constexpr float kFreeSpaceWeight = 0.15f;
  for (const PixelData& pixel : accumulated_pixel_data) {
    // TODO(whess): Take into account submap rotation.
    // TODO(whess): Document the approach and make it more independent from the
    // chosen resolution.
    const float z_difference = pixel.count > 0 ? pixel.max_z - pixel.min_z : 0;
    if (z_difference < kMinZDifference) {
      cell_data.push_back(0);  // value
      cell_data.push_back(0);  // alpha
      continue;
    }
    const float free_space = std::max(z_difference - pixel.count, 0.f);
    const float free_space_weight = kFreeSpaceWeight * free_space;
    const float total_weight = pixel.count + free_space_weight;
    const float free_space_probability = 1.f - pixel.max_probability;
    const float average_probability = ClampProbability(
        (pixel.probability_sum + free_space_probability * free_space_weight) /
        total_weight);
    const int delta = 128 - ProbabilityToLogOddsInteger(average_probability);
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cell_data.push_back(value);                         // value
    cell_data.push_back((value || alpha) ? alpha : 1);  // alpha
  }
  return cell_data;
}

void AddToTextureProto(
    const HybridGrid& hybrid_grid, const transform::Rigid3d& global_submap_pose,
    proto::SubmapQuery::Response::SubmapTexture* const texture) {
  // Generate an X-ray view through the 'hybrid_grid', aligned to the
  // xy-plane in the global map frame.
  const float resolution = hybrid_grid.resolution();
  texture->set_resolution(resolution);

  // Compute a bounding box for the texture.
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ExtractVoxelData(hybrid_grid, global_submap_pose.cast<float>(),
                       &min_index, &max_index);

  const int width = max_index.y() - min_index.y() + 1;
  const int height = max_index.x() - min_index.x() + 1;
  texture->set_width(width);
  texture->set_height(height);

  const std::vector<PixelData> accumulated_pixel_data = AccumulatePixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);
  const std::string cell_data = ComputePixelValues(accumulated_pixel_data);

  common::FastGzipString(cell_data, texture->mutable_cells());
  *texture->mutable_slice_pose() = transform::ToProto(
      global_submap_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(
          max_index.x() * resolution, max_index.y() * resolution,
          global_submap_pose.translation().z())));
}

void AddToTextureProto(
    const HybridGridTSDF& hybrid_grid,
    const transform::Rigid3d& global_submap_pose,
    proto::SubmapQuery::Response::SubmapTexture* const texture) {
  // Generate an X-ray view through the 'hybrid_grid', aligned to the
  // xy-plane in the global map frame.
  const float resolution = hybrid_grid.resolution();
  texture->set_resolution(resolution);

  // Compute a bounding box for the texture.
  Eigen::Array2i min_index(INT_MAX, INT_MAX);
  Eigen::Array2i max_index(INT_MIN, INT_MIN);
  const std::vector<Eigen::Array4i> voxel_indices_and_probabilities =
      ExtractVoxelData(hybrid_grid, global_submap_pose.cast<float>(),
                       &min_index, &max_index);

  const int width = max_index.y() - min_index.y() + 1;
  const int height = max_index.x() - min_index.x() + 1;
  texture->set_width(width);
  texture->set_height(height);

  const std::vector<PixelData> accumulated_pixel_data = AccumulatePixelData(
      width, height, min_index, max_index, voxel_indices_and_probabilities);
  const std::string cell_data = ComputePixelValues(accumulated_pixel_data);

  common::FastGzipString(cell_data, texture->mutable_cells());
  *texture->mutable_slice_pose() = transform::ToProto(
      global_submap_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(
          max_index.x() * resolution, max_index.y() * resolution,
          global_submap_pose.translation().z())));
}

}  // namespace

proto::SubmapsOptions3D CreateSubmapsOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::SubmapsOptions3D options;
  options.set_high_resolution(
      parameter_dictionary->GetDouble("high_resolution"));
  options.set_high_resolution_max_range(
      parameter_dictionary->GetDouble("high_resolution_max_range"));
  options.set_low_resolution(parameter_dictionary->GetDouble("low_resolution"));
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_high_resolution_range_data_inserter_options() =
      CreateRangeDataInserterOptions3D(
          parameter_dictionary
              ->GetDictionary("high_resolution_range_data_inserter")
              .get());
  *options.mutable_low_resolution_range_data_inserter_options() =
      CreateRangeDataInserterOptions3D(
          parameter_dictionary
              ->GetDictionary("low_resolution_range_data_inserter")
              .get());
  CHECK_GT(options.num_range_data(), 0);
  const std::string grid_type_string =
      parameter_dictionary->GetString("grid_type");
  proto::SubmapsOptions3D_GridType grid_type;
  CHECK(proto::SubmapsOptions3D_GridType_Parse(grid_type_string, &grid_type))
      << "Unknown SubmapsOptions3D_GridType kind: " << grid_type_string;
  options.set_grid_type(grid_type);
  return options;
}

Submap3D::Submap3D(const transform::Rigid3d& local_submap_pose,
                   std::unique_ptr<GridInterface> low_resolution_grid,
                   std::unique_ptr<GridInterface> high_resolution_grid,
                   const Eigen::VectorXf& rotational_scan_matcher_histogram,
                   ValueConversionTables* conversion_tables,
                   const common::Time& start_time)
    : Submap(local_submap_pose, start_time),
      rotational_scan_matcher_histogram_(rotational_scan_matcher_histogram),
      conversion_tables_(conversion_tables) {
  low_resolution_grid_ = std::move(low_resolution_grid);
  high_resolution_grid_ = std::move(high_resolution_grid);
}

Submap3D::Submap3D(const proto::Submap3D& proto,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_tables)  {
  UpdateFromProto(proto);
}

proto::Submap Submap3D::ToProto(
    const bool include_probability_grid_data) const {
  proto::Submap proto;
  auto* const submap_3d = proto.mutable_submap_3d();
  *submap_3d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_3d->set_num_range_data(num_range_data());
  submap_3d->set_finished(insertion_finished());
  if (include_probability_grid_data) {
    switch (low_resolution_grid_->GetGridType()) {
      case GridType::PROBABILITY_GRID: {
        *submap_3d->mutable_high_resolution_hybrid_grid() =
            static_cast<HybridGrid*>(high_resolution_grid_.get())->ToProto();
        *submap_3d->mutable_low_resolution_hybrid_grid() =
            static_cast<HybridGrid*>(low_resolution_grid_.get())->ToProto();
        break;
      }
      case GridType::TSDF: {
        *submap_3d->mutable_low_resolution_hybrid_grid_tsdf() =
            static_cast<HybridGridTSDF*>(low_resolution_grid_.get())->ToProto();
        *submap_3d->mutable_high_resolution_hybrid_grid_tsdf() =
            static_cast<HybridGridTSDF*>(high_resolution_grid_.get())
                ->ToProto();
        break;
      }
      case GridType::NONE:
        LOG(FATAL) << "Gridtype not initialized.";
        break;
    }
  }
  return proto;
}

void Submap3D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_3d());
  UpdateFromProto(proto.submap_3d());
}

void Submap3D::UpdateFromProto(const proto::Submap3D& submap_3d) {
  set_num_range_data(submap_3d.num_range_data());
  set_insertion_finished(submap_3d.finished());
  if (submap_3d.has_high_resolution_hybrid_grid()) {
    high_resolution_grid_ = absl::make_unique<HybridGrid>(
        submap_3d.high_resolution_hybrid_grid(), conversion_tables_);
  }
  if (submap_3d.has_high_resolution_hybrid_grid_tsdf()) {
    high_resolution_grid_ = absl::make_unique<HybridGridTSDF>(
        submap_3d.high_resolution_hybrid_grid_tsdf(), conversion_tables_);
  }
  if (submap_3d.has_low_resolution_hybrid_grid()) {
    low_resolution_grid_ = absl::make_unique<HybridGrid>(
        submap_3d.low_resolution_hybrid_grid(), conversion_tables_);
  }
  if (submap_3d.has_low_resolution_hybrid_grid_tsdf()) {
    low_resolution_grid_ = absl::make_unique<HybridGridTSDF>(
        submap_3d.low_resolution_hybrid_grid_tsdf(), conversion_tables_);
  }
  rotational_scan_matcher_histogram_ =
      Eigen::VectorXf::Zero(submap_3d.rotational_scan_matcher_histogram_size());
  for (Eigen::VectorXf::Index i = 0;
       i != submap_3d.rotational_scan_matcher_histogram_size(); ++i) {
    rotational_scan_matcher_histogram_(i) =
        submap_3d.rotational_scan_matcher_histogram(i);
  }
}

void Submap3D::ToResponseProto(
    const transform::Rigid3d& global_submap_pose,
    proto::SubmapQuery::Response* const response) const {
  response->set_submap_version(num_range_data());
  switch (low_resolution_grid_->GetGridType()) {
    case GridType::PROBABILITY_GRID: {
      HybridGrid* low_res_grid =
          static_cast<HybridGrid*>(low_resolution_grid_.get());
      HybridGrid* high_res_grid =
          static_cast<HybridGrid*>(high_resolution_grid_.get());
      AddToTextureProto(*low_res_grid, global_submap_pose,
                        response->add_textures());
      AddToTextureProto(*high_res_grid, global_submap_pose,
                        response->add_textures());
      break;
    }
    case GridType::TSDF: {
      HybridGridTSDF* low_res_grid =
          static_cast<HybridGridTSDF*>(low_resolution_grid_.get());
      HybridGridTSDF* high_res_grid =
          static_cast<HybridGridTSDF*>(high_resolution_grid_.get());
      AddToTextureProto(*low_res_grid, global_submap_pose,
                        response->add_textures());
      AddToTextureProto(*high_res_grid, global_submap_pose,
                        response->add_textures());
      break;
    }
    case GridType::NONE:
      LOG(FATAL) << "Gridtype not initialized.";
      break;
  }
}

void Submap3D::InsertData(
    const sensor::RangeData& range_data_in_local,
    const RangeDataInserterInterface* high_resolution_range_data_inserter,
    const RangeDataInserterInterface* low_resolution_range_data_inserter,
    const float high_resolution_max_range,
    const Eigen::Quaterniond& local_from_gravity_aligned,
    const Eigen::VectorXf& scan_histogram_in_gravity) {
  CHECK(!insertion_finished());
  // Transform range data into submap frame.
  const sensor::RangeData transformed_range_data = sensor::TransformRangeData(
      range_data_in_local, local_pose().inverse().cast<float>());

  // todo(kdaun) remove high_resolution_max_range parameter from Submap options
  // --> move to range_data_inserter
  high_resolution_range_data_inserter->Insert(transformed_range_data,
                                              high_resolution_grid_.get());

  low_resolution_range_data_inserter->Insert(transformed_range_data,
                                             low_resolution_grid_.get());
  set_num_range_data(num_range_data() + 1);
  const float yaw_in_submap_from_gravity = transform::GetYaw(
      local_pose().inverse().rotation() * local_from_gravity_aligned);
  rotational_scan_matcher_histogram_ +=
      scan_matching::RotationalScanMatcher::RotateHistogram(
          scan_histogram_in_gravity, yaw_in_submap_from_gravity);
}

void Submap3D::Finish() {
  CHECK(!insertion_finished());
  set_insertion_finished(true);
}

std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps3D::CreateRangeDataInserter(
    const proto::RangeDataInserterOptions3D& inserter_options) {
  switch (inserter_options.range_data_inserter_type()) {
    case proto::RangeDataInserterOptions3D::PROBABILITY_GRID_INSERTER_3D:
      return absl::make_unique<OccupancyGridRangeDataInserter3D>(
          inserter_options);
    case proto::RangeDataInserterOptions3D::TSDF_INSERTER_3D:
      return absl::make_unique<TSDFRangeDataInserter3D>(
          inserter_options.tsdf_range_data_inserter_options_3d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
      return absl::make_unique<TSDFRangeDataInserter3D>(
          inserter_options
              .tsdf_range_data_inserter_options_3d());  // todo(kdaun) fix
                                                        // error: control
                                                        // reaches end of
                                                        // non-void function
  }
}

ActiveSubmaps3D::ActiveSubmaps3D(const proto::SubmapsOptions3D& options)
    : options_(options),
      high_resolution_range_data_inserter_(CreateRangeDataInserter(
          options.high_resolution_range_data_inserter_options())),
      low_resolution_range_data_inserter_(CreateRangeDataInserter(
          options.low_resolution_range_data_inserter_options())) {}

std::vector<std::shared_ptr<const Submap3D>> ActiveSubmaps3D::submaps() const {
  return std::vector<std::shared_ptr<const Submap3D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap3D>> ActiveSubmaps3D::InsertData(
    const sensor::RangeData& range_data,
    const Eigen::Quaterniond& local_from_gravity_aligned,
    const Eigen::VectorXf& rotational_scan_matcher_histogram_in_gravity,
    const common::Time& time) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(transform::Rigid3d(range_data.origin.cast<double>(),
                                 local_from_gravity_aligned),
              rotational_scan_matcher_histogram_in_gravity.size(), time);
  }
  for (auto& submap : submaps_) {
    submap->InsertData(range_data, high_resolution_range_data_inserter_.get(),
                       low_resolution_range_data_inserter_.get(),
                       options_.high_resolution_max_range(),
                       local_from_gravity_aligned,
                       rotational_scan_matcher_histogram_in_gravity);
  }
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<GridInterface> ActiveSubmaps3D::CreateGrid(float resolution) {
  switch (options_.grid_type()) {
    case proto::SubmapsOptions3D_GridType_PROBABILITY_GRID:
      return absl::make_unique<HybridGrid>(resolution, &conversion_tables_);
    case proto::SubmapsOptions3D_GridType_TSDF:
      CHECK_EQ(options_.high_resolution_range_data_inserter_options()
                   .tsdf_range_data_inserter_options_3d()
                   .relative_truncation_distance(),
               options_.low_resolution_range_data_inserter_options()
                   .tsdf_range_data_inserter_options_3d()
                   .relative_truncation_distance());
      CHECK_EQ(options_.high_resolution_range_data_inserter_options()
                   .tsdf_range_data_inserter_options_3d()
                   .maximum_weight(),
               options_.low_resolution_range_data_inserter_options()
                   .tsdf_range_data_inserter_options_3d()
                   .maximum_weight());
      return absl::make_unique<HybridGridTSDF>(
          resolution,
          options_.high_resolution_range_data_inserter_options()
              .tsdf_range_data_inserter_options_3d()
              .relative_truncation_distance(),
          options_.low_resolution_range_data_inserter_options()
              .tsdf_range_data_inserter_options_3d()
              .maximum_weight(),
          &conversion_tables_);
    default:
      LOG(FATAL) << "Unknown GridType.";
      return absl::make_unique<HybridGridTSDF>(resolution, 0.3f, 1000.0f,
                                               &conversion_tables_); //todo(kdaun) fix default return
  }
}

void ActiveSubmaps3D::AddSubmap(
    const transform::Rigid3d& local_submap_pose,
    const int rotational_scan_matcher_histogram_size,
    const common::Time& time) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  const Eigen::VectorXf initial_rotational_scan_matcher_histogram =
      Eigen::VectorXf::Zero(rotational_scan_matcher_histogram_size);
  submaps_.emplace_back(new Submap3D(
      local_submap_pose,
      std::unique_ptr<GridInterface>(static_cast<GridInterface*>(
          CreateGrid(options_.low_resolution()).release())),
      std::unique_ptr<GridInterface>(static_cast<GridInterface*>(
          CreateGrid(options_.high_resolution()).release())),
      initial_rotational_scan_matcher_histogram, &conversion_tables_, time));
}

}  // namespace mapping
}  // namespace cartographer
