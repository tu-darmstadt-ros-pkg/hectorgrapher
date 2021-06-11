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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TSDF_MULTI_RESOLUTION_SPACE_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TSDF_MULTI_RESOLUTION_SPACE_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_multi_resolution_tsdf.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'hybrid_grid' with a
// 'translation' and 'rotation'. The cost increases when points fall into less
// occupied space, i.e. at voxels with lower values.
template <typename PointCloudType>
class MultiResolutionTSDFSpaceCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const PointCloudType& point_cloud,
      const std::vector<const HybridGridTSDF*>& tsdf) {
    return new ceres::AutoDiffCostFunction<
        MultiResolutionTSDFSpaceCostFunction3D, ceres::DYNAMIC /* residuals */,
        3 /* translation variables */, 4 /* rotation variables */>(
        new MultiResolutionTSDFSpaceCostFunction3D(scaling_factor, point_cloud,
                                                   tsdf),
        point_cloud.size());
  }

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

 private:
  MultiResolutionTSDFSpaceCostFunction3D(
      const double scaling_factor, const PointCloudType& point_cloud,
      const std::vector<const HybridGridTSDF*>& tsdf)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(tsdf) {}

  MultiResolutionTSDFSpaceCostFunction3D(
      const MultiResolutionTSDFSpaceCostFunction3D&) = delete;
  MultiResolutionTSDFSpaceCostFunction3D& operator=(
      const MultiResolutionTSDFSpaceCostFunction3D&) = delete;

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
    //"TODO(kdaun) check for interpolation artifacts at TSDF band boundaries";
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> world =
          transform * point_cloud_[i].position.template cast<T>();
      const T tsd = interpolated_grid_.GetTSD(world[0], world[1], world[2]);
      residual[i] = scaling_factor_ * tsd;
    }
    return true;
  }

  const double scaling_factor_;
  const PointCloudType& point_cloud_;
  const InterpolatedMultiResolutionTSDF interpolated_grid_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // tsdf->CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_TSDF_MULTI_RESOLUTION_SPACE_COST_FUNCTION_3D_H_
