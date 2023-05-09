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

#include "cartographer/mapping/internal/3d/motion_model/motion_model.h"

namespace cartographer {
namespace mapping {
MotionModel::MotionModel()
    : last_state_(State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d::Zero())),
      last_state_time_(common::FromUniversal(0)),
      initialized_(false) {}
void MotionModel::initialize(const State& initial_state,
                             const common::Time& time) {
  last_state_ = initial_state;
  last_state_time_ = time;
  initialized_ = true;
}

State MotionModel::initialize(const common::Time& time) {
  last_state_time_ = time;
  initialized_ = true;
  return last_state_;
}

MotionModel::~MotionModel() {}
bool MotionModel::isInitialized() { return initialized_; }

State MotionModel::ExtrapolateState(const common::Time& time) const {
  State res = last_state_;
  //  LOG(INFO)<<"estimated orientation "<<last_state_;
  return res;
}

bool MotionModel::HasDataUntil(const common::Time& time) const { return true; }
bool MotionModel::HasDataBefore(const common::Time& time) const { return true; }

void MotionModel::UpdateState(const State& state, const common::Time& time) {
  last_state_ = state;
  last_state_time_ = time;
  deleteUntilBefore(time);
}

transform::Rigid3d MotionModel::RelativeTransform(const common::Time& t0,
                                                  const common::Time& t1) {
  return transform::Rigid3d::Identity();
}

void MotionModel::AddIMUData(const sensor::ImuData& imu_data) {
  imu_buffer_.push_back(imu_data);
}

void MotionModel::AddOdometryData(const sensor::OdometryData& odom_data) {
  odom_buffer_.push_back(odom_data);
}

void MotionModel::deleteUntilBefore(const common::Time& time) {
  while (imu_buffer_.size() > 1 && imu_buffer_[1].time < time) {
    imu_buffer_.pop_front();
  }
  while (odom_buffer_.size() > 1 && odom_buffer_[1].time < time) {
    odom_buffer_.pop_front();
  }
}

}  // namespace mapping
}  // namespace cartographer
