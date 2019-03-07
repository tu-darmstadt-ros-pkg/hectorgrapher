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

#include "cartographer/mapping/internal/2d/scan_matching/fast_esdf_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

#include "cairo.h"
#include "cartographer/evaluation/marching_squares.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {
void renderGrid(const cartographer::mapping::TSDF2D& grid) {
  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_y_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_x_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context, r, g, b);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  time_t seconds;
  time(&seconds);
  std::string filename = "esdf_" + std::to_string(seconds) + ".png";
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}
}  // namespace

FastESDFScanMatcher2D::FastESDFScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options), limits_(grid.limits()) {
  const TSDF2D& tsdf = static_cast<const TSDF2D&>(grid);
  max_depth_ = options.branch_and_bound_depth();
  precomputation_grid_ = absl::make_unique<TSDF2D>(
      CreateESDFFromTSDF(std::pow(options.branch_and_bound_depth(), 2) *
                             grid.limits().resolution(),
                         tsdf.conversion_tables_, tsdf));
  // renderGrid(tsdf);
  // renderGrid(*precomputation_grid_.get());
}

FastESDFScanMatcher2D::~FastESDFScanMatcher2D() {}

bool FastESDFScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float max_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, max_score, score,
                                   pose_estimate);
}

bool FastESDFScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float max_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  // Compute a search window around the center of the submap that includes it
  // fully.
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() -
      0.5 * limits_.resolution() *
          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                          limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   max_score, score, pose_estimate);
}

bool FastESDFScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, float max_score, float* score,
    transform::Rigid2d* pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);
  const Candidate2D best_candidate =
      BranchAndBound(discrete_scans, search_parameters,
                     lowest_resolution_candidates, max_depth_, max_score);
  if (best_candidate.score < max_score) {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

std::vector<Candidate2D>
FastESDFScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  float search_bound_delta =
      max_depth_ > 1
          ? std::pow(3, max_depth_ - 2) * std::sqrt(4.5) *
                precomputation_grid_->limits().resolution()
          : 0.f;
  //  std::pow(3, max_depth_) * precomputation_grid_->limits().resolution();
  ScoreCandidates(*precomputation_grid_.get(), discrete_scans,
                  search_parameters, &lowest_resolution_candidates,
                  search_bound_delta);
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>
FastESDFScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = std::pow(3, max_depth_ - 1);
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastESDFScanMatcher2D::ScoreCandidates(
    const TSDF2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates, float search_bound) const {
  for (Candidate2D& candidate : *candidates) {
    float sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      float update =
          std::max(std::abs(precomputation_grid.GetTSD(proposed_xy_index)) -
                       search_bound,
                   0.f);
      sum += update;
    }
    candidate.score =
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size());
  }
  std::sort(candidates->begin(), candidates->end(), std::less<Candidate2D>());
}

Candidate2D FastESDFScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float max_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = max_score;

  // LOG(INFO)<<"Depth  "<<candidate_depth <<"\t"<<std::pow(3, candidate_depth -
  // 1);
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score >= max_score) {
      break;
    }
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = std::pow(3, candidate_depth - 1);
    for (int x_offset : {-half_width, 0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {-half_width, 0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    float search_bound_delta =
        candidate_depth > 1
            ? std::pow(3, candidate_depth - 2) * std::sqrt(4.5) *
                  precomputation_grid_->limits().resolution()
            : 0.f;
    ScoreCandidates(*precomputation_grid_.get(), discrete_scans,
                    search_parameters, &higher_resolution_candidates,
                    search_bound_delta);
    best_high_resolution_candidate = std::min(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }

  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
