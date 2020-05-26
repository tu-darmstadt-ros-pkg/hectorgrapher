#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PLANE_FEATURE_COST_FUNCTOR_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PLANE_FEATURE_COST_FUNCTOR_3D_H_

#include <cmath>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping {

// Computes the cost of rotating 'rotation_quaternion' to 'target_rotation'.
// Cost increases with the solution's distance from 'target_rotation'.
class PlaneFeatureCostFunctor3D {
 public:
  PlaneFeatureCostFunctor3D(
      const double scaling_factor,
      const sensor::TimedPointCloud& lhs_plane_features,
      const sensor::TimedPointCloud& rhs_plane_features,
      const std::vector<std::vector<int>>& correspondences)
      : scaling_factor_(scaling_factor),
        lhs_plane_features_(lhs_plane_features),
        rhs_plane_features_(rhs_plane_features),
        correspondences_(correspondences) {}

  template <typename T>
  bool operator()(const T* const translation_0, const T* const rotation_0,
                  const T* const translation_1, const T* const rotation_1,
                  T* const residual) const {
    const transform::Rigid3<T> transform_0(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation_0),
        Eigen::Quaternion<T>(rotation_0[0], rotation_0[1], rotation_0[2],
                             rotation_0[3]));
    const transform::Rigid3<T> transform_1(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation_1),
        Eigen::Quaternion<T>(rotation_1[0], rotation_1[1], rotation_1[2],
                             rotation_1[3]));

    for (size_t i = 0; i < correspondences_.size(); ++i) {
      CHECK_GT(correspondences_.size(), i);
      CHECK_EQ(correspondences_[i].size(), 4);
      CHECK(rhs_plane_features_.size() > correspondences_[i][0]);
      CHECK(lhs_plane_features_.size() > correspondences_[i][1]);
      CHECK(lhs_plane_features_.size() > correspondences_[i][2]);
      CHECK(lhs_plane_features_.size() > correspondences_[i][3]);
      const Eigen::Matrix<T, 3, 1> rhs_i =
          transform_1 *
          rhs_plane_features_[correspondences_[i][0]].position.cast<T>();
      const Eigen::Matrix<T, 3, 1> lhs_j =
          transform_0 *
          lhs_plane_features_[correspondences_[i][1]].position.cast<T>();
      const Eigen::Matrix<T, 3, 1> lhs_l =
          transform_0 *
          lhs_plane_features_[correspondences_[i][2]].position.cast<T>();
      const Eigen::Matrix<T, 3, 1> lhs_m =
          transform_0 *
          lhs_plane_features_[correspondences_[i][3]].position.cast<T>();

      const T numerator =
          (rhs_i - lhs_j).dot((lhs_j - lhs_l).cross(lhs_j - lhs_m));
      const T denominator =
          ((lhs_j - lhs_l).cross(lhs_j - lhs_m)).norm() + T(1e-7);
      residual[i] = scaling_factor_ * numerator / denominator;
    }
    return true;
  }

 private:
  PlaneFeatureCostFunctor3D(const PlaneFeatureCostFunctor3D&) = delete;
  PlaneFeatureCostFunctor3D& operator=(const PlaneFeatureCostFunctor3D&) =
      delete;

  const double scaling_factor_;
  const sensor::TimedPointCloud& lhs_plane_features_;
  const sensor::TimedPointCloud& rhs_plane_features_;
  const std::vector<std::vector<int>>& correspondences_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_PLANE_FEATURE_COST_FUNCTOR_3D_H_
