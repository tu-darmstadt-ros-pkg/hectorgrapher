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

#include "cartographer/mapping/internal/3d/motion_model/imu_motion_model.h"

namespace cartographer {
namespace mapping {


State IMUMotionModel::initialize(const common::Time& time) {
  last_state_time_ = time;
  last_state_ = stateAt(time, true);
  initialized_ = true;
  return last_state_;
}

State IMUMotionModel::ExtrapolateState(const common::Time& time) const {
  if(time == last_state_time_) return last_state_;
  bool has_enclosing_data = imu_buffer_.size() > 1 &&
                            imu_buffer_.back().time >= time &&
                            imu_buffer_.front().time <= last_state_time_;
  State state_t0 = stateAt(last_state_time_, has_enclosing_data);
  State state_t1 = stateAt(time, has_enclosing_data);
  transform::Rigid3d t0 =
      transform::Rigid3d(state_t0.Translation(), state_t0.Rotation());
  transform::Rigid3d t1 =
      transform::Rigid3d(state_t1.Translation(), state_t1.Rotation());
  transform::Rigid3d delta = t0.inverse() * t1;
  transform::Rigid3d t_last =
      transform::Rigid3d(last_state_.Translation(), last_state_.Rotation());
  transform::Rigid3d prediction = t_last * delta;
  State res = State(prediction.translation(), prediction.rotation(),
                    Eigen::Vector3d::Identity());
  // TODO res.time_state_ = time;
  return res;
}

State IMUMotionModel::stateAt(const common::Time& time,
                              bool use_imu_data) const {
  double dt = common::ToSeconds(time - last_state_time_);
  Eigen::Vector3d pos =
      last_state_.Translation() + dt * last_state_.LinearVelocity();
  State res = last_state_;
  //  res.time_state_ = time;
  res.translation = {{pos.x(), pos.y(), pos.z()}};
  if (use_imu_data) {
    if (imu_buffer_.empty()) {
      LOG(WARNING) << "IMU buffer empty defaulting to identity";
      return res;
    }
    if (imu_buffer_.size() == 1) {
      LOG(WARNING) << "Only single message in IMU buffer.";
      return res;
    }
    if (time <= imu_buffer_.front().time) {
      LOG(WARNING) << "Request before available data";
      return res;
    }
    if (time >= imu_buffer_.back().time) {
      LOG(WARNING) << "Request after available data";
      return res;
    }

    for (auto it = imu_buffer_.begin(); it != imu_buffer_.end() - 1; ++it) {
      if (it->time <= time && std::next(it)->time >= time) {
        const auto& it0 = it;
        auto it1 = std::next(it);
        double interpolation_factor = common::ToSeconds(time - it0->time) /
                                      common::ToSeconds(it1->time - it0->time);
        Eigen::Quaterniond rotation =
            it0->rotation.slerp(interpolation_factor, it1->rotation);
        // TODO compute AngularVelocity estimate
        return {pos, rotation, last_state_.LinearVelocity()};  // TODO
                                                               // last_state_.AngularLinearVelocity(),
                                                               // time};
      }
    }
  }

  return {pos, last_state_.Rotation(),
          last_state_
              .LinearVelocity()};  // TODO last_state_.AngularLinearVelocity(),
                                   // time};
}

bool IMUMotionModel::HasDataUntil(const common::Time& time) const {
  return imu_buffer_.size() > 1 && imu_buffer_.back().time >= time;
}

bool IMUMotionModel::HasDataBefore(const common::Time& time) const {
  return imu_buffer_.size() > 1 && imu_buffer_.front().time <= time;
}

void IMUMotionModel::UpdateState(const State& state, const common::Time& time) {
  double dt = common::ToSeconds(time - last_state_time_);
  Eigen::Vector3d velocity =
      (1.0 / dt) * (state.Translation() - last_state_.Translation());
  if (dt < 0.001) {
    velocity = last_state_.LinearVelocity();
  } else if (dt < 0.1) {
    double interpolation_ratio = dt / 0.1;
    velocity = interpolation_ratio * velocity +
               (1.0 - interpolation_ratio) * last_state_.LinearVelocity();
  }
  // TODO angular velocity
  //   Eigen::Quaterniond dq = last_state_.Rotation().inverse()*
  //   state.Rotation();
  //  Eigen::Vector3d angular_velocity = (1.0/dt)*(state.Translation() -
  //  last_state_.Translation());

  last_state_ = {
      state.Translation(), state.Rotation(),
      velocity};  // TODO state.AngularLinearVelocity(), state.time_state_};

  deleteUntilBefore(time);
  buffer_t0_idx = -1;
  buffer_t1_idx = -1;
}

transform::Rigid3d IMUMotionModel::RelativeTransform(const common::Time& t0,
                                                     const common::Time& t1) {
  Eigen::Quaterniond rotation_t0;
  Eigen::Quaterniond rotation_t1;
  int t0_idx = 0;
  bool found_t0 = false;
  if (buffer_t0_idx > 0) {
    if (buffer_t0_time == t0) {
      rotation_t0 = buffer_rotation_t0;
      t0_idx = buffer_t0_idx;
      found_t0 = true;
      num_hits++;
    } else if (buffer_t0_time < t0) {
      t0_idx = buffer_t0_idx;
      num_continues++;
    } else {
      num_research++;
    }
  } else {
    num_no_buffer++;
  }
  if (!found_t0) {
    for (; t0_idx < imu_buffer_.size() - 1; ++t0_idx) {
      if (imu_buffer_[t0_idx].time <= t0 &&
          imu_buffer_[t0_idx + 1].time >= t0) {
        const auto& data_a = imu_buffer_[t0_idx];
        const auto& data_b = imu_buffer_[t0_idx + 1];
        double interpolation_factor =
            common::ToSeconds(t0 - data_a.time) /
            common::ToSeconds(data_b.time - data_a.time);
        rotation_t0 =
            data_a.rotation.slerp(interpolation_factor, data_b.rotation);
        break;
      }
    }
  }
  buffer_t0_idx = t0_idx;
  buffer_t0_time = t0;
  buffer_rotation_t0 = rotation_t0;

  int t1_idx = t0_idx;
  bool found_t1 = false;
  if (buffer_t1_idx > 0) {
    if (buffer_t1_time == t1) {
      rotation_t1 = buffer_rotation_t1;
      t1_idx = buffer_t1_idx;
      found_t1 = true;
      num_hits++;
    } else if (buffer_t1_time < t1) {
      t1_idx = buffer_t1_idx;
      num_continues++;
    } else {
      num_research++;
    }
  } else {
    num_no_buffer++;
  }
  if (!found_t1) {
    for (; t1_idx < imu_buffer_.size() - 1; ++t1_idx) {
      if (imu_buffer_[t1_idx].time <= t1 &&
          imu_buffer_[t1_idx + 1].time >= t1) {
        const auto& data_a = imu_buffer_[t1_idx];
        //        const auto& data_b = imu_buffer_[t1_idx + 1];
        //        double interpolation_factor =
        //            (t1 - data_a.time).toSec() /
        //            (data_b.time - data_a.time).toSec();
        //        rotation_t1 =
        //            ToQuaterniond(data_a.orientation)
        //                .slerp(interpolation_factor,
        //                ToQuaterniond(data_b.orientation));
        //
        rotation_t1 = data_a.rotation;
        break;
      }
    }
  }
  buffer_t1_idx = t1_idx;
  buffer_t1_time = t1;
  buffer_rotation_t1 = rotation_t1;

  //  if((num_hits + num_continues + num_research + num_no_buffer + 2) % 1000000
  //  == 0) {
  //    ROS_INFO("hit ratio %i \t %i \t %i \t %i", num_hits, num_continues,
  //    num_research, num_no_buffer);
  //  }

  double dt = common::ToSeconds(t1 - t0);
  Eigen::Vector3d pos =
      last_state_.Translation() + dt * last_state_.LinearVelocity();

  return transform::Rigid3d::Rotation(rotation_t0.inverse() * rotation_t1);
}
}  // namespace mapping
}  // namespace cartographer
