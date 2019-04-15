/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"
#include "cartographer/evaluation/marching_squares.h"
#include "cartographer/evaluation/grid_drawer.h"
#include "cairo.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;
// Minimum distance between range observation and origin. Otherwise, range
// observations are discarded.
constexpr float kMinRangeMeters = 1e-6f;
const float kSqrtTwoPi = std::sqrt(2.0 * M_PI);

void GrowAsNeeded(const sensor::RangeData& range_data,
                  const float truncation_distance, TSDF2D* const tsdf) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    const Eigen::Vector3f direction =
        (hit.position - range_data.origin).normalized();
    const Eigen::Vector3f end_position =
        hit.position + truncation_distance * direction;
    bounding_box.extend(end_position.head<2>());
  }
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  //  constexpr float kPadding = 1e-6f;
  constexpr float kPadding = 5;
  tsdf->GrowLimits(bounding_box.min() - kPadding * Eigen::Vector2f::Ones());
  tsdf->GrowLimits(bounding_box.max() + kPadding * Eigen::Vector2f::Ones());
}

float GaussianKernel(const float x, const float sigma) {
  return 1.0 / (kSqrtTwoPi * sigma) * std::exp(-0.5 * x * x / (sigma * sigma));
}

std::pair<Eigen::Array2i, Eigen::Array2i> SuperscaleRay(
    const Eigen::Vector2f& begin, const Eigen::Vector2f& end,
    TSDF2D* const tsdf) {
  const MapLimits& limits = tsdf->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));

  const Eigen::Array2i superscaled_begin =
      superscaled_limits.GetCellIndex(begin);
  const Eigen::Array2i superscaled_end = superscaled_limits.GetCellIndex(end);
  return std::make_pair(superscaled_begin, superscaled_end);
}

struct RangeDataSorter {
  RangeDataSorter(Eigen::Vector3f origin) { origin_ = origin.head<2>(); }
  bool operator()(const sensor::RangefinderPoint& lhs,
                  const sensor::RangefinderPoint& rhs) {
    const Eigen::Vector2f delta_lhs =
        (lhs.position.head<2>() - origin_).normalized();
    const Eigen::Vector2f delta_rhs =
        (rhs.position.head<2>() - origin_).normalized();
    if ((delta_lhs[1] < 0.f) != (delta_rhs[1] < 0.f)) {
      return delta_lhs[1] < 0.f;
    } else if (delta_lhs[1] < 0.f) {
      return delta_lhs[0] < delta_rhs[0];
    } else {
      return delta_lhs[0] > delta_rhs[0];
    }
  }

 private:
  Eigen::Vector2f origin_;
};

float ComputeRangeWeightFactor(float range, int exponent) {
  float weight = 0.f;
  if (std::abs(range) > kMinRangeMeters) {
    weight = 1.f / (std::pow(range, exponent));
  }
  return weight;
}

float WeightedMeanOfTwoAngles(float a, float wa, float b, float wb) {
  return std::atan2(wa * sin(a) + wb * sin(b), wa * cos(a) + wb * cos(b));
}
}  // namespace

proto::TSDFRangeDataInserterOptions2D CreateTSDFRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::TSDFRangeDataInserterOptions2D options;
  options.set_truncation_distance(
      parameter_dictionary->GetDouble("truncation_distance"));
  options.set_maximum_weight(parameter_dictionary->GetDouble("maximum_weight"));
  options.set_update_free_space(
      parameter_dictionary->GetBool("update_free_space"));
  *options
       .mutable_normal_estimation_options() = CreateNormalEstimationOptions2D(
      parameter_dictionary->GetDictionary("normal_estimation_options").get());
  options.set_project_sdf_distance_to_scan_normal(
      parameter_dictionary->GetBool("project_sdf_distance_to_scan_normal"));
  options.set_update_weight_range_exponent(
      parameter_dictionary->GetInt("update_weight_range_exponent"));
  options.set_update_weight_angle_scan_normal_to_ray_kernel_bandwith(
      parameter_dictionary->GetDouble(
          "update_weight_angle_scan_normal_to_ray_kernel_bandwith"));
  options.set_update_weight_distance_cell_to_hit_kernel_bandwith(
      parameter_dictionary->GetDouble(
          "update_weight_distance_cell_to_hit_kernel_bandwith"));
  options.set_free_space_weight(
      parameter_dictionary->GetDouble("free_space_weight"));
  options.set_min_normal_weight(
      parameter_dictionary->GetDouble("min_normal_weight"));
  return options;
}

TSDFRangeDataInserter2D::TSDFRangeDataInserter2D(
    const proto::TSDFRangeDataInserterOptions2D& options)
    : options_(options) {}

// Casts a ray from origin towards hit for each hit in range data.
// If 'options.update_free_space' is 'true', all cells along the ray
// until 'truncation_distance' behind hit are updated. Otherwise, only the cells
// within 'truncation_distance' around hit are updated.
void TSDFRangeDataInserter2D::Insert(const sensor::RangeData& range_data,
                                     GridInterface* grid) const {
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  TSDF2D* tsdf = static_cast<TSDF2D*>(grid);
  GrowAsNeeded(range_data, truncation_distance, tsdf);

  // Compute normals if needed.
  bool scale_update_weight_angle_scan_normal_to_ray =
      options_.update_weight_angle_scan_normal_to_ray_kernel_bandwith() != 0.f;
  sensor::RangeData sorted_range_data = range_data;
  std::vector<float> normals;
  std::vector<float> weights;

  std::vector<std::pair<float, float>> scan_normals;
  std::vector<std::pair<float, float>> tsdf_normals;
  std::vector<std::pair<float, float>> scan_render_normals;
  std::vector<std::pair<float, float>> tsdf_render_normals;
  std::vector<std::pair<float, float>> combined_render_normals;

  float max_scans = options_.normal_estimation_options().num_normal_samples();
  if (options_.project_sdf_distance_to_scan_normal() ||
      scale_update_weight_angle_scan_normal_to_ray) {
    if (options_.normal_estimation_options().sort_range_data()) {
      std::sort(sorted_range_data.returns.begin(),
                sorted_range_data.returns.end(),
                RangeDataSorter(sorted_range_data.origin));
    }
    scan_normals = EstimateNormals(
        sorted_range_data, options_.normal_estimation_options());
    tsdf_normals =
        EstimateNormalsFromTSDF(sorted_range_data, *tsdf);

    for (size_t hit_index = 0; hit_index < sorted_range_data.returns.size();
         ++hit_index) {
      const Eigen::Vector2f hit =
          sorted_range_data.returns[hit_index].position.head<2>();

      bool use_tsdf_normals = true;
      if (tsdf_normals[hit_index].second ==
              0.f ||  !use_tsdf_normals) {
        normals.push_back(scan_normals[hit_index].first);
        //        if (hit_index % 2000 == 0) LOG(INFO) << "pass";
        float scan_weight = std::max(
            std::min(float(scan_normals[hit_index].second), max_scans) /
                max_scans,
            float(options_.min_normal_weight()));
        weights.push_back(scan_weight);
      } else {
        float scan_weight = std::max(
            std::min(float(scan_normals[hit_index].second), max_scans) /
                max_scans,
            float(options_.min_normal_weight()));
        float tsdf_weight = options_.normal_estimation_options().tsdf_weight_scale() * std::min(float (tsdf_normals[hit_index].second)/tsdf->value_converter_->getMaxWeight(), 1.f);
        float const_weight = options_.normal_estimation_options().const_weight();
        float ratio = tsdf_weight / (scan_weight + tsdf_weight + const_weight);
        float normal =
            WeightedMeanOfTwoAngles(scan_normals[hit_index].first, 1.f - ratio,
                                    tsdf_normals[hit_index].first, ratio);
        normals.push_back(normal);
        weights.push_back(scan_weight * (1.f - ratio) + tsdf_weight * ratio);
      }

      //      float scan_weight =
      //      std::min(float(scan_normals[hit_index].second)/20.f, 1.f);
      //      float tsdf_weight = std::min(float(tsdf_normals[hit_index].second)
      //      /
      //                                       tsdf->value_converter_->getMaxWeight(),
      //                                   1.f);
      //      float tsdf_min_offset = 0.0;
      //      tsdf_weight = std::max(
      //          0.f, options_.normal_estimation_options().tsdf_weight_scale()
      //          *
      //                   (tsdf_weight - tsdf_min_offset) / (1.f -
      //                   tsdf_min_offset));
      //      float const_weight =
      //      options_.normal_estimation_options().const_weight();
      //      float ratio = tsdf_weight / (scan_weight + tsdf_weight +
      //      const_weight);
      //      float normal =
      //          WeightedMeanOfTwoAngles(scan_normals[hit_index].first, 1.f -
      //          ratio,
      //                                  tsdf_normals[hit_index].first, ratio);
      //      tsdf_render_normals.emplace_back(tsdf_normals[hit_index].first,
      //      0.f);
      //      scan_render_normals.emplace_back(scan_normals[hit_index].first,
      //      1.0f);
      //      combined_render_normals.emplace_back(normal, 0.5f);
    }
  }
//  evaluation::GridDrawer drawer_scan_normals(tsdf->limits());
//  drawer_scan_normals.DrawTSD(*tsdf);
//  drawer_scan_normals.DrawPointcloud(
//                range_data.returns,
//                transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
//                transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)));
//  drawer_scan_normals.DrawWeightedNormals(scan_normals, sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)), 20);

  //  evaluation::GridDrawer drawer_tsdf_normals(tsdf->limits());
  ////  drawer_tsdf_normals.DrawTSD(*tsdf);
  //  drawer_tsdf_normals.DrawIsoSurface(*tsdf);
  //  drawer_tsdf_normals.DrawPointcloud(
  //      range_data.returns,
  //      transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
  //      transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)));
  //  drawer_tsdf_normals.DrawWeightedNormals(tsdf_render_normals,
  //  sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
  //  1.f);
  //  drawer_tsdf_normals.DrawWeightedNormals(scan_render_normals,
  //  sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
  //  1.f);
  //  drawer_tsdf_normals.DrawWeightedNormals(combined_render_normals,
  //  sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
  //  1.f);
  //  auto start = std::chrono::high_resolution_clock::now();
  //  std::string timestamp=
  //      std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
  //          start.time_since_epoch())
  //                         .count());
  ////  drawer_scan_normals.ToFile("scan_normals_" + timestamp + ".png");
  //  drawer_tsdf_normals.ToFile("tsdf_normals_" + timestamp + ".png");
  //

     static int update_index = 0;
      update_index++;
      if (update_index % 150 == 0 && update_index > 1) {

            LOG(INFO)<<"DRAWING";
        evaluation::GridDrawer drawer(tsdf->limits());
      drawer.DrawTSD(*tsdf);
      drawer.DrawIsoSurface(*tsdf);

        auto start = std::chrono::high_resolution_clock::now();
        std::string timestamp=
            std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                start.time_since_epoch())
                               .count());
      //  drawer_scan_normals.ToFile("scan_normals_" + timestamp + ".png");
      drawer.ToFile("tsdf_" + timestamp + ".png");

        LOG(INFO)<<"DONE";
      }

  const Eigen::Vector2f origin = sorted_range_data.origin.head<2>();
  for (size_t hit_index = 0; hit_index < sorted_range_data.returns.size();
       ++hit_index) {
    const Eigen::Vector2f hit =
        sorted_range_data.returns[hit_index].position.head<2>();
    const float normal = normals.empty()
                             ? std::numeric_limits<float>::quiet_NaN()
                             : normals[hit_index];
    float normal_weight = weights.empty()
                              ? std::numeric_limits<float>::quiet_NaN()
                              : weights[hit_index];
    InsertHit(options_, hit, origin, normal, tsdf, normal_weight);
  }
  tsdf->FinishUpdate();

//  evaluation::GridDrawer drawer_scan_normals(tsdf->limits());
//  drawer_scan_normals.DrawTSD(*tsdf);
////  drawer_scan_normals.DrawPointcloud(
////                range_data.returns,
////                transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
////                transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)));
//  drawer_scan_normals.DrawWeightedNormals(scan_normals, sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)), 20);
//
//  evaluation::GridDrawer drawer_tsdf_normals(tsdf->limits());
//  drawer_tsdf_normals.DrawTSD(*tsdf);
//  drawer_tsdf_normals.DrawIsoSurface(*tsdf);
////  drawer_tsdf_normals.DrawPointcloud(
////      range_data.returns,
////      transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)),
////      transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)));
//  drawer_tsdf_normals.DrawWeightedNormals(tsdf_normals, sorted_range_data, transform::Rigid2d({0.0,0.0}, Eigen::Rotation2Dd(0.0)), tsdf->value_converter_->getMaxWeight());
//  auto start = std::chrono::high_resolution_clock::now();
//  std::string timestamp=
//              std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
//                  start.time_since_epoch())
//                                 .count());
//  drawer_scan_normals.ToFile("scan_normals_"+timestamp+".png");
//  drawer_tsdf_normals.ToFile("tsdf_normals_"+timestamp+".png");

}

void TSDFRangeDataInserter2D::InsertHit(
    const proto::TSDFRangeDataInserterOptions2D& options,
    const Eigen::Vector2f& hit, const Eigen::Vector2f& origin, float normal,
    TSDF2D* tsdf, float normal_weight = 1.f) const {

  if(!options_.project_sdf_distance_to_scan_normal())
  {
    normal_weight = 1.f;
  }

  if (normal_weight == 0.f) return;
  const Eigen::Vector2f ray = hit - origin;
  const float range = ray.norm();
  const float truncation_distance =
      static_cast<float>(options_.truncation_distance());
  const float truncation_ratio = truncation_distance / range;
  Eigen::Vector2f ray_begin = range < truncation_distance
                                  ? origin
                                  : origin + (1.0f - truncation_ratio) * ray;

  Eigen::Vector2f ray_end = origin + (1.0f + truncation_ratio) * ray;

  std::pair<Eigen::Array2i, Eigen::Array2i> superscaled_freespace_ray =
      SuperscaleRay(origin, origin + (1.0f - truncation_ratio) * ray, tsdf);
  std::vector<Eigen::Array2i> ray_freespace_mask =
      RayToPixelMask(superscaled_freespace_ray.first,
                     superscaled_freespace_ray.second, kSubpixelScale);

  Eigen::Vector2f normal_vector;
  if (options_.project_sdf_distance_to_scan_normal()) {
    normal_vector = Eigen::Vector2f({cos(normal), sin(normal)});
    ray_begin = hit - truncation_distance * normal_vector;
    ray_end = hit + truncation_distance * normal_vector;
  }

  std::pair<Eigen::Array2i, Eigen::Array2i> superscaled_ray =
      SuperscaleRay(ray_begin, ray_end, tsdf);
  std::vector<Eigen::Array2i> ray_mask = RayToPixelMask(
      superscaled_ray.first, superscaled_ray.second, kSubpixelScale);

  // Precompute weight factors.
  float weight_factor_angle_ray_normal = 1.f;
  if (options_.update_weight_angle_scan_normal_to_ray_kernel_bandwith() !=
      0.f) {
    const Eigen::Vector2f negative_ray = -ray;
    float angle_ray_normal =
        common::NormalizeAngleDifference(normal - common::atan2(negative_ray));
    weight_factor_angle_ray_normal = std::cos(angle_ray_normal);
  }
  float weight_factor_range = 1.f;
  if (options_.update_weight_range_exponent() != 0) {
    weight_factor_range = ComputeRangeWeightFactor(
        range, options_.update_weight_range_exponent());
  }

  // Update Cells.
  for (const Eigen::Array2i& cell_index : ray_mask) {
    // if (tsdf->CellIsUpdated(cell_index)) continue;
    Eigen::Vector2f cell_center = tsdf->limits().GetCellCenter(cell_index);
    float distance_cell_to_origin = (cell_center - origin).norm();
    float update_tsd = range - distance_cell_to_origin;
    if (options.project_sdf_distance_to_scan_normal()) {
      update_tsd = (cell_center - hit).dot(normal_vector);
    }

    update_tsd =
        common::Clamp(update_tsd, -truncation_distance, truncation_distance);
    float update_weight = weight_factor_range * weight_factor_angle_ray_normal;

    update_weight *= normal_weight;
    if (options_.update_weight_distance_cell_to_hit_kernel_bandwith() != 0.f) {
      float d = update_tsd;  // update_tsd;  // for some reason this works
                             // better than range -
      d = range - distance_cell_to_origin;
      // distance_cell_to_origin TODO(kdaun) Understand.
      // std::max(double(std::abs(range - distance_cell_to_origin)),
      // options.truncation_distance());//update_tsd;//range -
      // distance_cell_to_origin;
      float exp_update = std::exp(
          -d * d *
          options_.update_weight_distance_cell_to_hit_kernel_bandwith());
      update_weight *= exp_update;
    }
    UpdateCell(cell_index, update_tsd, update_weight, tsdf);
  }

  if(options_.update_free_space()){
  for (const Eigen::Array2i& cell_index : ray_freespace_mask) {
    // if (tsdf->CellIsUpdated(cell_index)) continue;
    Eigen::Vector2f cell_center = tsdf->limits().GetCellCenter(cell_index);
    float distance_cell_to_origin = (cell_center - origin).norm();
    float update_tsd = range - distance_cell_to_origin;
    if (options.project_sdf_distance_to_scan_normal()) {
      update_tsd = (cell_center - hit).dot(normal_vector);
    }
    if (update_tsd < truncation_distance) continue;
    update_tsd = truncation_distance;
    float update_weight = options_.free_space_weight() * weight_factor_range *
                          weight_factor_angle_ray_normal;
    update_weight *= normal_weight;
    UpdateCell(cell_index, update_tsd, update_weight, tsdf);
  }
  }
}

void TSDFRangeDataInserter2D::UpdateCell(const Eigen::Array2i& cell,
                                         float update_sdf, float update_weight,
                                         TSDF2D* tsdf) const {
  if (update_weight == 0.f) return;
  const std::pair<float, float> tsd_and_weight = tsdf->GetTSDAndWeight(cell);
  float updated_weight = tsd_and_weight.second + update_weight;
  float updated_sdf = (tsd_and_weight.first * tsd_and_weight.second +
                       update_sdf * update_weight) /
                      updated_weight;
  updated_weight =
      std::min(updated_weight, static_cast<float>(options_.maximum_weight()));
  tsdf->SetCell(cell, updated_sdf, updated_weight);
}

}  // namespace mapping
}  // namespace cartographer
