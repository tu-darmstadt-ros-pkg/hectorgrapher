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

#ifndef CARTOGRAPHER_MAPPING_3D_GRAVITY_DIRECTION_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_GRAVITY_DIRECTION_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

class GravityDirectionCostFunction {
 public:
  GravityDirectionCostFunction(
      const double scaling_factor,
      const Eigen::Quaterniond& gravity_direction_in_tracking)
      : scaling_factor_(scaling_factor),
        gravity_direction_in_tracking_(gravity_direction_in_tracking) {}

  GravityDirectionCostFunction(const GravityDirectionCostFunction&) = delete;
  GravityDirectionCostFunction& operator=(const GravityDirectionCostFunction&) =
      delete;

  template <typename T>
  bool operator()(const T* const orientation, T* residual) const {
    Eigen::Quaternion<T> qorientation(orientation[0], orientation[1],
                                      orientation[2], orientation[3]);

    Eigen::Quaternion<T> delta =
        gravity_direction_in_tracking_.cast<T>() * qorientation;
    residual[0] = scaling_factor_ * transform::GetRoll(delta);
    residual[1] = scaling_factor_ * transform::GetPitch(delta);
    return true;
  }

 private:
  const double scaling_factor_;
  const Eigen::Quaterniond gravity_direction_in_tracking_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_GRAVITY_DIRECTION_COST_FUNCTION_H_