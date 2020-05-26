#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_EDGE_FEATURE_COST_FUNCTOR_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_EDGE_FEATURE_COST_FUNCTOR_3D_H_

#include <cmath>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping {

// Computes the cost of rotating 'rotation_quaternion' to 'target_rotation'.
// Cost increases with the solution's distance from 'target_rotation'.
class EdgeFeatureCostFunctor3D {
 public:
  EdgeFeatureCostFunctor3D(const double scaling_factor,
                           const sensor::TimedPointCloud& lhs_edge_features,
                           const sensor::TimedPointCloud& rhs_edge_features,
                           const std::vector<std::vector<int>>& correspondences)
      : scaling_factor_(scaling_factor),
        lhs_edge_features_(lhs_edge_features),
        rhs_edge_features_(rhs_edge_features),
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
      CHECK_EQ(correspondences_[i].size(), 3);
      CHECK(rhs_edge_features_.size() > correspondences_[i][0]);
      CHECK(lhs_edge_features_.size() > correspondences_[i][1]);
      CHECK(lhs_edge_features_.size() > correspondences_[i][2]);
      const Eigen::Matrix<T, 3, 1> rhs_i =
          transform_1 *
          rhs_edge_features_[correspondences_[i][0]].position.cast<T>();
      const Eigen::Matrix<T, 3, 1> lhs_j =
          transform_0 *
          lhs_edge_features_[correspondences_[i][1]].position.cast<T>();
      const Eigen::Matrix<T, 3, 1> lhs_l =
          transform_0 *
          lhs_edge_features_[correspondences_[i][2]].position.cast<T>();

      if(((rhs_i - lhs_j).norm() < T(1e-7)) || ((rhs_i - lhs_l).norm() < T(1e-7)) || ((lhs_j - lhs_l).norm() < T(1e-7))) {
        residual[i] = T(scaling_factor_ * 1e-7);
      }
      else {
        const T numerator = ((rhs_i - lhs_j).cross(rhs_i - lhs_l)).norm();
        const T denominator = (lhs_j - lhs_l).norm() + T(1e-7);

//      if((numerator / denominator) < T(1e-5)) {
//        LOG(INFO)<<"\n DEGRADED RESIDUAL "<<numerator / denominator
//        <<"\n numerator "<<numerator <<" denominator "<<denominator
//        <<"\n i "<<i<<" "<<"/ "<<correspondences_.size()-1
//        <<"\n idxi "<<i<<" "<<"/ "<<correspondences_.size()-1
//        <<"\n idxj "<<i<<" "<<"/ "<<correspondences_.size()-1
//        <<"\n idxl "<<i<<" "<<"/ "<<correspondences_.size()-1
//        <<"\n rhs_i "<<rhs_i
//        <<"\n lhs_j "<<lhs_j
//        <<"\n lhs_l "<<lhs_l;
//      }

        residual[i] = scaling_factor_ * numerator / denominator;
      }
    }
    return true;
  }

 private:
  EdgeFeatureCostFunctor3D(const EdgeFeatureCostFunctor3D&) = delete;
  EdgeFeatureCostFunctor3D& operator=(const EdgeFeatureCostFunctor3D&) = delete;

  const double scaling_factor_;
  const sensor::TimedPointCloud& lhs_edge_features_;
  const sensor::TimedPointCloud& rhs_edge_features_;
  const std::vector<std::vector<int>>& correspondences_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_EDGE_FEATURE_COST_FUNCTOR_3D_H_
