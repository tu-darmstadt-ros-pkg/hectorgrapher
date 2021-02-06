#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_GNC_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_GNC_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "cartographer/mapping/internal/2d/scan_matching/gnc_iteration_callback.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include "cartographer/mapping/internal/2d/scan_matching/logging_iteration_callback.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

//proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
//    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
class CeresScanMatcherGnc2D : public CeresScanMatcher2D {
 public:
  explicit CeresScanMatcherGnc2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcherGnc2D();

  CeresScanMatcherGnc2D(const CeresScanMatcherGnc2D&) = delete;
  CeresScanMatcherGnc2D& operator=(const CeresScanMatcherGnc2D&) = delete;

  void newMatch(unsigned long new_size) const;
  void newMatch(unsigned long new_size, float start_convexity) const;

  virtual void Match(const Eigen::Vector2d& target_translation,
                     const transform::Rigid2d& initial_pose_estimate,
                     const sensor::PointCloud& point_cloud, const Grid2D& grid,
                     transform::Rigid2d* pose_estimate,
                     ceres::Solver::Summary* summary) const;

  virtual void Evaluate(const Eigen::Vector2d& target_translation,
                        const transform::Rigid2d& initial_pose_estimate,
                        const sensor::PointCloud& point_cloud,
                        const Grid2D& grid,
                        double* cost, std::vector<double>* residuals,
                        std::vector<double>* jacobians) const;

  GncIterationCallback* get_gnc_callback();

 protected:
  void setupProblem(const Eigen::Vector2d& target_translation,
                    const transform::Rigid2d& initial_pose_estimate,
                    const sensor::PointCloud& point_cloud, const Grid2D& grid,
                    double* ceres_pose_estimate, ceres::Problem* problem) const;
  mutable GncIterationCallback gnc_weight_update;
  mutable LoggingIterationCallback logging_callback;
  int max_retries;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_GNC_2D_H_
