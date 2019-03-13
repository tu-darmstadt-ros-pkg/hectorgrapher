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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable after
// construction.

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FAST_ESDF_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FAST_ESDF_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_scan_matcher_interface_2d.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

struct BBEvaluatedCandidates {
  double x;
  double y;
  float score;
  float search_bound;
};

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class FastESDFScanMatcher2D : public FastScanMatcherInterface2D {
 public:
  FastESDFScanMatcher2D(
      const Grid2D& grid,
      const proto::FastCorrelativeScanMatcherOptions2D& options);
  ~FastESDFScanMatcher2D();

  FastESDFScanMatcher2D(const FastESDFScanMatcher2D&) = delete;
  FastESDFScanMatcher2D& operator=(const FastESDFScanMatcher2D&) = delete;

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  bool Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, float max_score,
             float* score, transform::Rigid2d* pose_estimate) const override;

  // Aligns 'point_cloud' within the full 'grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float max_score,
                       float* score,
                       transform::Rigid2d* pose_estimate) const override;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float max_score, float* score,
      transform::Rigid2d* pose_estimate) const;
  std::vector<Candidate2D> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters,
      std::vector<BBEvaluatedCandidates>& bb_regions) const;
  std::vector<Candidate2D> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const TSDF2D& precomputation_grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* const candidates,
                       float search_bound,
                       std::vector<BBEvaluatedCandidates>& bb_regions) const;
  Candidate2D BranchAndBound(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters,
      const std::vector<Candidate2D>& candidates, int candidate_depth,
      float max_score, std::vector<BBEvaluatedCandidates>& bb_regions) const;

  const proto::FastCorrelativeScanMatcherOptions2D options_;
  MapLimits limits_;
  std::unique_ptr<TSDF2D> precomputation_grid_;
  int max_depth_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_FAST_ESDF_SCAN_MATCHER_2D_H_
