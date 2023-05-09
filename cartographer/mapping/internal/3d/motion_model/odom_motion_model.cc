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

#include "cartographer/mapping/internal/3d/motion_model/odom_motion_model.h"

namespace cartographer {
namespace mapping {

State OdomMotionModel::initialize(const common::Time& time) {
  last_state_ = stateAt(time);
  last_state_time_ = time;
  initialized_ = true;
  return last_state_;
}
State OdomMotionModel::ExtrapolateState(const common::Time& time) const {
  if (time == last_state_time_) return last_state_;
  State odom_t0 = stateAt(last_state_time_);
  State odom_t1 = stateAt(time);
  transform::Rigid3d t0 = odom_t0.ToRigid();
  transform::Rigid3d t1 = odom_t1.ToRigid();
  transform::Rigid3d delta = t0.inverse() * t1;
  transform::Rigid3d t_last = last_state_.ToRigid();
  transform::Rigid3d prediction = t_last * delta;
  State res = State(prediction.translation(), prediction.rotation(),
                    Eigen::Vector3d::Identity());
  return res;
}

State OdomMotionModel::stateAt(const common::Time& time) const {
  if (odom_buffer_.empty()) {
    LOG(WARNING) << "Odom buffer empty defaulting to identity";
    return State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 Eigen::Vector3d::Zero());
  }
  if (odom_buffer_.size() == 1) {
    LOG(WARNING) << "Only single message in buffer.";
    return State(odom_buffer_.front().pose.translation(),
                 odom_buffer_.front().pose.rotation(),
                 odom_buffer_.front().angular_velocity);
  }
  if (time <= odom_buffer_.front().time) {
    LOG(WARNING) << "Request before available data. Delta = %f",
        common::ToSeconds(time - odom_buffer_.front().time);
    return State(odom_buffer_.front().pose.translation(),
                 odom_buffer_.front().pose.rotation(),
                 odom_buffer_.front().angular_velocity);
  }
  if (time >= odom_buffer_.back().time) {
    LOG(WARNING) << "Request after available data.  Delta = %f",
        common::ToSeconds(odom_buffer_.back().time - time);
    return State(odom_buffer_.back().pose.translation(),
                 odom_buffer_.back().pose.rotation(),
                 odom_buffer_.back().angular_velocity);
  }

  for (auto it = odom_buffer_.begin(); it != odom_buffer_.end() - 1; ++it) {
    if (it->time <= time && std::next(it)->time >= time) {
      const auto& it0 = it;
      auto it1 = std::next(it);
      double interpolation_factor = common::ToSeconds(time - it0->time) /
                                    common::ToSeconds(it1->time - it0->time);
      Eigen::Quaterniond rotation = it0->pose.rotation().slerp(
          interpolation_factor, it1->pose.rotation());
      Eigen::Vector3d position =
          (1.0 - interpolation_factor) * it0->pose.translation() +
          interpolation_factor * it1->pose.translation();
      Eigen::Vector3d velocity =
          (1.0 - interpolation_factor) * it0->linear_velocity +
          interpolation_factor * it1->linear_velocity;
      Eigen::Vector3d angular_velocity =
          (1.0 - interpolation_factor) * it0->angular_velocity +
          interpolation_factor * it1->angular_velocity;
      return {position, rotation, velocity};  // TODO {angular_velocity, time};
    }
  }
  LOG(ERROR) << "no odom message found";
  return State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
               Eigen::Vector3d::Zero());
}

bool OdomMotionModel::HasDataUntil(const common::Time& time) const {
  return odom_buffer_.size() > 1 && odom_buffer_.back().time >= time;
}

bool OdomMotionModel::HasDataBefore(const common::Time& time) const {
  return odom_buffer_.size() > 1 && odom_buffer_.front().time <= time;
}

transform::Rigid3d OdomMotionModel::RelativeTransform(const common::Time& t0,
                                                      const common::Time& t1) {
  State odom_t0 = stateAt(t0);
  State odom_t1 = stateAt(t1);
  transform::Rigid3d transform_t0 = odom_t0.ToRigid();
  transform::Rigid3d transform_t1 = odom_t1.ToRigid();
  transform::Rigid3d delta = transform_t0.inverse() * transform_t1;
  return delta;
}
}  // namespace mapping
}  // namespace cartographer