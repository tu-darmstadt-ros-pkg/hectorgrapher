#ifndef CARTOGRAPHER_FAST_SCAN_MATCHER_INTERFACE_2D_H
#define CARTOGRAPHER_FAST_SCAN_MATCHER_INTERFACE_2D_H

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

class FastScanMatcherInterface2D {
 public:
  virtual ~FastScanMatcherInterface2D(){};

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  virtual bool Match(const transform::Rigid2d& initial_pose_estimate,
                     const sensor::PointCloud& point_cloud, float min_score,
                     float* score, transform::Rigid2d* pose_estimate) const = 0;

  // Aligns 'point_cloud' within the full 'grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  virtual bool MatchFullSubmap(const sensor::PointCloud& point_cloud,
                               float min_score, float* score,
                               transform::Rigid2d* pose_estimate) const = 0;
};

}  // scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_FAST_SCAN_MATCHER_INTERFACE_2D_H
