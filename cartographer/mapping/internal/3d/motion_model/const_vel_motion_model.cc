//=================================================================================================
// Copyright (c) 2023, Kevin Daun, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "cartographer/mapping/internal/3d/motion_model/const_vel_motion_model.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
State ConstVelMotionModel::ExtrapolateState(const common::Time& time) const {
  double dt = common::ToSeconds(time - last_state_time_);
  Eigen::Vector3d pos =
      last_state_.Translation() + dt * last_state_.LinearVelocity();
  Eigen::Vector3d integrated_angular_velocity =
      dt * last_state_.AngularVelocity();
  Eigen::Quaterniond orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          integrated_angular_velocity) *
      last_state_.Rotation();
  return {pos, orientation, last_state_.LinearVelocity()};  // TODO
                                                            // Eigen::Vector3d(last_state_.angular_velocity.data()),
                                                            // time};
}

bool ConstVelMotionModel::HasDataUntil(const common::Time& time) const {
  return true;
}
bool ConstVelMotionModel::HasDataBefore(const common::Time& time) const {
  return true;
}

void ConstVelMotionModel::UpdateState(const State& state,
                                      const common::Time& time) {
  double dt = common::ToSeconds(time - last_state_time_);
  if (dt <= 0.0) {
    return;
  }
  Eigen::Vector3d velocity =
      (1.0 / dt) * (state.Translation() - last_state_.Translation());
  velocity = 0.5 * velocity + 0.5 * last_state_.LinearVelocity();
  Eigen::Quaterniond dq = last_state_.Rotation().inverse() * state.Rotation();
  Eigen::Vector3d angular_velocity =
      transform::RotationQuaternionToAngleAxisVector(dq) / dt;
  angular_velocity =
      0.5 * angular_velocity + 0.5 * last_state_.AngularVelocity();
  last_state_ = {state.Translation(), state.Rotation(),
                 velocity};  // TODO angular_velocity, state.time_state_};
}

transform::Rigid3d ConstVelMotionModel::RelativeTransform(
    const common::Time& t0, const common::Time& t1) {
  double dt = common::ToSeconds(t1 - t0);
  Eigen::Vector3d position = dt * last_state_.LinearVelocity();
  Eigen::Vector3d integrated_angular_velocity =
      dt * last_state_.AngularVelocity();
  Eigen::Quaterniond orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          integrated_angular_velocity);
  return {position, orientation};
}
}  // namespace mapping
}  // namespace cartographer