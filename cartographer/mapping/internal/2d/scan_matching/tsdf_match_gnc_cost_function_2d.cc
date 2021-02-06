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

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/internal/2d/scan_matching/interpolated_tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "cartographer/mapping/internal/2d/scan_matching/gnc_iteration_callback.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

//struct ComputeDistValueFunctor {
//    bool operator()(const double* r2, double* value) const {
////      LOG(INFO) << "Writing in Adress: " << global_gnc_state;
//      global_gnc_state->set_distance(current_pos, *r2);
//      return true;
//    }
//    const GncIterationCallback* global_gnc_state;
//    mutable unsigned long current_pos;
//};

// Computes a cost for matching the 'point_cloud' in the 'grid' at
// a 'pose'. The cost increases with the signed distance of the matched point
// location in the 'grid'.
class TSDFMatchGncCostFunction2D {
 public:
    TSDFMatchGncCostFunction2D(const double empty_space_cost,
                               const double residual_scaling_factor,
                               const sensor::PointCloud& point_cloud,
                               const TSDF2D& grid,
                               const GncIterationCallback* gnc_state)
      : empty_space_cost_(empty_space_cost),
        max_weight_(grid.value_converter_->getMaxWeight()),
        residual_scaling_factor_(residual_scaling_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(grid),
        gnc_state(gnc_state) {}
//        functor_(new ComputeDistValueFunctor{.global_gnc_state = gnc_state}){
////      ComputeDistValueFunctor* functor_ = new ComputeDistValueFunctor;
//      compute_dist.reset(new ceres::CostFunctionToFunctor<1, 1>(
//          new ceres::NumericDiffCostFunction<ComputeDistValueFunctor,
//              ceres::CENTRAL,
//              1,
//              1>(functor_)));
////      LOG(INFO) << "GNC Adress: " << gnc_state;
//    }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    const Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    const Eigen::Rotation2D<T> rotation(pose[2]);
    const Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);
//    std::cout << "Transform:" << transform << std::endl;
    T summed_weight = T(0);
    T res_sum = T(0);
    T max_residual = T(0);
//    LOG(INFO) << "X: " << GetScalarFromJet(pose[0])
//              << "Y: " << GetScalarFromJet(pose[1])
//              << "w: " << GetScalarFromJet(pose[2]);
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      T grid_weight = interpolated_grid_.GetWeight(world[0], world[1]);
      const T point_weight = grid_weight * T(1.0 - empty_space_cost_) +
                             T(empty_space_cost_) * max_weight_;
      summed_weight += point_weight;  // TODO
//      T p1 = interpolated_grid_.GetWeight(world[0], world[1]);
//      T p2 = interpolated_grid_.GetCorrespondenceCost(world[0], world[1]);
      residual[i] =
          T(point_cloud_.size()) * residual_scaling_factor_ *
          interpolated_grid_.GetCorrespondenceCost(world[0], world[1]) *
          point_weight;  // new weights from gnc //TODO
//      LOG(INFO) << gnc_state->get_weights()->at(i);
//      if (residual[i] > T(0)) residual[i] = residual[i] * residual[i];
//      else residual[i] = -(residual[i] * residual[i]);

//      if (max_residual < residual[i]) {
//        max_residual = residual[i];
////        LOG(INFO) << "Residual: " << i << ": " << GetScalarFromJet(residual[i]);
//      }

      // OLD METHOD:
//      gnc_state->set_distance(i, abs(GetScalarFromJet(residual[i])));
      // NEW METHOD:
//      gnc_state->set_distance(i, abs(GetScalarFromJet(residual[i]) /
//                                     gnc_state->get_weights()->at(i)));

//      T f = T(0);
//      functor_->current_pos = i;
//      (*compute_dist)(&residual[i], &f);
//      LOG(INFO) << "Residual: " << i << ": " << GetScalarFromJet(residual[i]);
    }
//    LOG(INFO) << "Residuals: " << GetScalarFromJet(residual[0])
//              << ", " << GetScalarFromJet(residual[1]) << ", "
//              << GetScalarFromJet(residual[2]) << ", "
//              << GetScalarFromJet(residual[3]);
//    LOG(INFO) << "Weights: " << gnc_state->get_weights()->at(0)
//              << ", " << gnc_state->get_weights()->at(1) << ", "
//              << gnc_state->get_weights()->at(2) << ", "
//              << gnc_state->get_weights()->at(3);

    // OLD METHOD:
//    gnc_state->set_max_residual(abs(pow(GetScalarFromJet(max_residual), 1)));
    // NEW:
//    gnc_state->set_max_residual(abs(pow(GetScalarFromJet(max_residual / summed_weight), 1)));
    if (summed_weight == T(0)) return false;
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
//      residual[i] /= summed_weight;

      residual[i] = residual[i] * gnc_state->get_weights()->at(i) / summed_weight;
      gnc_state->set_distance(i, abs(GetScalarFromJet(residual[i]) /
                                     gnc_state->get_weights()->at(i)));
      if (max_residual < residual[i]) {
        max_residual = residual[i];
//        LOG(INFO) << "Residual: " << i << ": " << GetScalarFromJet(residual[i]);
      }

      // NEW:
//      gnc_state->set_distance(i, abs(GetScalarFromJet(residual[i])));
      res_sum += residual[i];
    }

    gnc_state->set_max_residual(abs(pow(GetScalarFromJet(max_residual), 1)));

//    LOG(INFO) << "Residual sum: " << res_sum << "PWeight: " << summed_weight;
//    gnc_state->set_max_residual(abs(pow(GetScalarFromJet(res_sum * summed_weight), 1)));

//    std::cout << summed_weight << " | " << res_sum << std::endl;
//    gnc_state->get_gm_gnc_shape();

      return true;
  }

 private:
  template <typename T>
  double GetScalarFromJet(const T& jet) const {
    return GetScalarFromJet(jet.a);
  }
  double GetScalarFromJet(const double jet) const {
    return double(jet);
  }
//  template <typename T>
//  double sgn(T val) {
//    return (T(0) < val) - (val < T(0));
//  }

  TSDFMatchGncCostFunction2D(const TSDFMatchGncCostFunction2D&) = delete;
  TSDFMatchGncCostFunction2D& operator=(const TSDFMatchGncCostFunction2D&) =
      delete;

  const double empty_space_cost_;
  const double max_weight_;
  const double residual_scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedTSDF2D interpolated_grid_;
  const GncIterationCallback* gnc_state;
//  ComputeDistValueFunctor* functor_;
//  std::unique_ptr<ceres::CostFunctionToFunctor<1, 1> > compute_dist;
};

ceres::CostFunction* CreateTSDFMatchGncCostFunction2D(
    const double empty_space_cost,
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const TSDF2D& tsdf, const GncIterationCallback* gnc_state) {
  return new ceres::AutoDiffCostFunction<TSDFMatchGncCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new TSDFMatchGncCostFunction2D(empty_space_cost, scaling_factor,
                                            point_cloud, tsdf, gnc_state),
                                            point_cloud.size());
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
