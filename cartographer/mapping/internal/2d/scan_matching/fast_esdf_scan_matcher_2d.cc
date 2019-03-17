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
#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/evaluation/marching_squares.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {


FastESDFScanMatcher2D::FastESDFScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options), limits_(grid.limits()) {
  const TSDF2D& tsdf = static_cast<const TSDF2D&>(grid);
  max_depth_ = options.branch_and_bound_depth();
  precomputation_grid_ = absl::make_unique<TSDF2D>(
      CreateESDFFromTSDF((std::pow(3, max_depth_ - 1) - 1) * std::sqrt(2) *
                             0.5 * grid.limits().resolution(),
                         tsdf.conversion_tables_, tsdf));
  //    evaluation::GridDrawer drawer(tsdf.limits());
  //    drawer.DrawTSD(tsdf);
  //    drawer.DrawIsoSurface(tsdf);
  //    drawer.DrawWeights(tsdf);
  //    auto start = std::chrono::high_resolution_clock::now();
  //    std::string filename =
  //        "grid_with_inserted_cloud" +
  //            std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
  //                start.time_since_epoch())
  //                               .count()) +
  //            ".png";
  //    drawer.ToFile(filename);

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
  // search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  std::vector<BBEvaluatedCandidates> bb_regions;
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters,
                                        bb_regions);

  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      max_depth_ - 1, max_score, bb_regions);

  if (best_candidate.score < max_score) {
//            evaluation::GridDrawer drawer(precomputation_grid_->limits());
//            drawer.DrawTSD(*precomputation_grid_);
//            drawer.DrawBBBounds(bb_regions, initial_pose_estimate);
//            drawer.DrawPointcloud(
//                point_cloud, initial_pose_estimate,
//                transform::Rigid2d(
//                    {initial_pose_estimate.translation().x() +
//                    best_candidate.x,
//                     initial_pose_estimate.translation().y() +
//                     best_candidate.y},
//                    initial_rotation *
//                    Eigen::Rotation2Dd(best_candidate.orientation)));
//            auto start = std::chrono::high_resolution_clock::now();
//            std::string filename =
//                "grid_with_inserted_cloud" +
//                std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
//                                   start.time_since_epoch())
//                                   .count()) +
//                ".png";
//            drawer.ToFile(filename);



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
    const SearchParameters& search_parameters,
    std::vector<BBEvaluatedCandidates>& bb_regions) const {
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  float search_bound_delta =
      max_depth_ > 1
          ? (std::pow(3, max_depth_ - 1) - 1) * std::sqrt(2) * 0.5 *
                precomputation_grid_->limits().resolution()
          : 0.f;
  // LOG(INFO)<<"depth "<<max_depth_<<" search_bound_delta
  // "<<search_bound_delta;
  ScoreCandidates(*precomputation_grid_.get(), discrete_scans,
                  search_parameters, &lowest_resolution_candidates,
                  search_bound_delta, bb_regions);
  return lowest_resolution_candidates;
}

std::vector<Candidate2D>
FastESDFScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = std::pow(3, max_depth_ - 1);
  int num_candidates = 0;

  std::vector<int> center_offsets_x;
  std::vector<int> center_offsets_y;
  center_offsets_x.reserve(search_parameters.num_scans);
  center_offsets_y.reserve(search_parameters.num_scans);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates = int(
        std::ceil(double(search_parameters.linear_bounds[scan_index].max_x -
                         search_parameters.linear_bounds[scan_index].min_x) /
                  linear_step_size));
    int center_offset_x = ((search_parameters.linear_bounds[scan_index].max_x -
                            search_parameters.linear_bounds[scan_index].min_x) %
                           linear_step_size) /
                          2;
    center_offsets_x.push_back(center_offset_x);
    const int num_lowest_resolution_linear_y_candidates = int(
        std::ceil(double(search_parameters.linear_bounds[scan_index].max_y -
                         search_parameters.linear_bounds[scan_index].min_y) /
                  linear_step_size));
    int center_offset_y = ((search_parameters.linear_bounds[scan_index].max_y -
                            search_parameters.linear_bounds[scan_index].min_y) %
                           linear_step_size) /
                          2;
    center_offsets_y.push_back(center_offset_y);
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset =
             search_parameters.linear_bounds[scan_index].min_x +
             center_offsets_x[scan_index];
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y +
               center_offsets_y[scan_index];
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
    std::vector<Candidate2D>* const candidates, float search_bound,
    std::vector<BBEvaluatedCandidates>& bb_regions) const {
  for (Candidate2D& candidate : *candidates) {
    float sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      float update = std::max(
          std::abs(precomputation_grid.GetTSD(proposed_xy_index)) -
              search_bound * 1.0825f,  // Correction for ESDF approximation
          0.f);
      sum += update;
    }
    candidate.score =
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size());
    bb_regions.push_back(
        {candidate.x, candidate.y, candidate.score, search_bound});
  }
  std::sort(candidates->begin(), candidates->end(), std::less<Candidate2D>());
}

Candidate2D FastESDFScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    float max_score, std::vector<BBEvaluatedCandidates>& bb_regions) const {
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
      int x_upper_bound_delta = x_offset > 0 ? x_offset / 2 : 3 * x_offset / 2;
      int x_lower_bound_delta = x_offset > 0 ? 3 * x_offset / 2 : x_offset / 2;
      if ((candidate.x_index_offset + x_upper_bound_delta >
           search_parameters.linear_bounds[candidate.scan_index].max_x) ||
          (candidate.x_index_offset + x_lower_bound_delta <
           search_parameters.linear_bounds[candidate.scan_index].min_x)) {
        continue;
      }
      for (int y_offset : {-half_width, 0, half_width}) {
        int y_upper_bound_delta =
            y_offset > 0 ? y_offset / 2 : 3 * y_offset / 2;
        int y_lower_bound_delta =
            y_offset > 0 ? 3 * y_offset / 2 : y_offset / 2;
        if ((candidate.y_index_offset + y_upper_bound_delta >
             search_parameters.linear_bounds[candidate.scan_index].max_y) ||
            (candidate.y_index_offset + y_lower_bound_delta <
             search_parameters.linear_bounds[candidate.scan_index].min_y)) {
          continue;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    float search_bound_delta =
        candidate_depth > 1
            ? (std::pow(3, candidate_depth - 1) - 1) * std::sqrt(2) * 0.5 *
                  precomputation_grid_->limits().resolution()
            : 0.f;
    ScoreCandidates(*precomputation_grid_.get(), discrete_scans,
                    search_parameters, &higher_resolution_candidates,
                    search_bound_delta, bb_regions);
    best_high_resolution_candidate = std::min(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score, bb_regions));
  }

  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
