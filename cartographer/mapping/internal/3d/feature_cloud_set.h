#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_FEATURE_CLOUD
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_FEATURE_CLOUD

#include <array>
#include <chrono>
#include <deque>
#include <memory>

#include "Open3D/Open3D.h"

#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/state.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/pose_extrapolator.h"

#include "cartographer/metrics/family_factory.h"

#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class FeatureCloudSet {
 public:
  FeatureCloudSet(float plane_threshold, float edge_threshold, size_t num_scans,
                  size_t num_segments)
      : plane_threshold_(plane_threshold),
        edge_threshold_(edge_threshold),
        num_scans_(num_scans),
        num_segments_(num_segments),
        multi_scan_cloud_(num_scans) {}
  FeatureCloudSet() {}

  void Transform(const transform::Rigid3f& transformation);

  void ComputeFeatureSet(
      const sensor::TimedPointCloudData& range_data_in_tracking);

  void ExtractCorrespondences(
      const sensor::TimedPointCloud& plane_feature_locations,
      const sensor::TimedPointCloud& edge_feature_locations,
      std::vector<std::vector<int>>* edge_correspondences,
      std::vector<std::vector<int>>* plane_correspondences);

  float plane_threshold_;
  float edge_threshold_;
  size_t num_scans_;
  size_t num_segments_;
  std::vector<sensor::TimedPointCloud> multi_scan_cloud_;
  sensor::TimedPointCloud plane_feature_locations_;
  sensor::TimedPointCloud edge_feature_locations_;
  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_edge_;
  std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_plane_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_FEATURE_CLOUD
