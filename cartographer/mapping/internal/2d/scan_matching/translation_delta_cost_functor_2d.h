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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes the cost of translating 'pose' to 'target_translation'.
// Cost increases with the solution's distance from 'target_translation'.
class TranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const double scaling_factor_vertical,
      const Eigen::Vector2d& target_translation, const double target_angle) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor,
                                          scaling_factor_vertical,
                                          target_translation, target_angle));
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // assuming Robot moves rather in forward direction, not to the side
    if ((bool)scaling_factor_vertical_) {
//      const Eigen::Rotation2D<T> rotation(pose[2]);
//      const Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
      Eigen::Matrix<T, 2, 2> rot_;
//      const double angle = angle_ ? angle_ : pose[2];
//      T angle = T(0) + angle_;  // Use starting angle, nut current angle
      T angle = pose[2];  // Use starting angle, nut current angle
      rot_ << cos(angle), sin(angle), -sin(angle), cos(angle);
      T diff_x = pose[0] - x_;
      T diff_y = pose[1] - y_;
      const Eigen::Matrix<T, 2, 1> point(diff_x, diff_y);
      const Eigen::Matrix<T, 2, 1> robot_frame = rot_ * point;
      //    LOG(INFO) << "X : " << GetScalarFromJet(pose[0]) << " Y : "
      //              << GetScalarFromJet(pose[1]);
      //    LOG(INFO) << "X*: " << GetScalarFromJet(world[0]) << " Y*: "
      //              << GetScalarFromJet(world[1]);
      residual[0] = scaling_factor_ * (robot_frame[0]);
      residual[1] = scaling_factor_vertical_ * (robot_frame[1]);
    } else {
      residual[0] = scaling_factor_ * (pose[0] - x_);
      residual[1] = scaling_factor_ * (pose[1] - y_);
    }
    return true;
  }

 private:
  template <typename T>  // TODO remove helper?
  double GetScalarFromJet(const T& jet) const {
    return GetScalarFromJet(jet.a);
  }
  double GetScalarFromJet(const double jet) const {
    return double(jet);
  }
  // Constructs a new TranslationDeltaCostFunctor2D from the given
  // 'target_translation' (x, y).
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const double scaling_factor_vertical,
      const Eigen::Vector2d& target_translation, const double target_angle)
      : scaling_factor_(scaling_factor),
        scaling_factor_vertical_(scaling_factor_vertical),
        x_(target_translation.x()),
        y_(target_translation.y()),
        angle_(target_angle) {}

  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d& target_translation)
      : scaling_factor_(scaling_factor),
        scaling_factor_vertical_(0.0),
        x_(target_translation.x()),
        y_(target_translation.y()),
        angle_(0.0) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D&) = delete;
  TranslationDeltaCostFunctor2D& operator=(
      const TranslationDeltaCostFunctor2D&) = delete;

  const double scaling_factor_;
  const double scaling_factor_vertical_;
  const double x_;
  const double y_;
  const double angle_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
