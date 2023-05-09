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

#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

#include <queue>

#include "cartographer/mapping/internal/3d/state.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"

namespace cartographer {
namespace mapping {
class MotionModel {
 public:
  MotionModel();
  virtual ~MotionModel();
  void initialize(const State& initial_state, const common::Time& time);
  virtual State initialize(const common::Time& time);
  bool isInitialized();
  virtual State ExtrapolateState(const common::Time& time) const;
  virtual bool HasDataUntil(const common::Time& time) const;
  virtual bool HasDataBefore(const common::Time& time) const;
  virtual void UpdateState(const State& state, const common::Time& time);
  virtual transform::Rigid3d RelativeTransform(const common::Time& t0,
                                               const common::Time& t1);
  void AddIMUData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odom_data);

 protected:
  // Deletes IMU and Odom buffer, keeping one message before time
  void deleteUntilBefore(const common::Time& time);
  std::deque<sensor::ImuData> imu_buffer_;
  std::deque<sensor::OdometryData> odom_buffer_;
  State last_state_;
  common::Time last_state_time_;
  bool initialized_;
};

}  // namespace mapping
}  // namespace cartographer
#endif  // MOTION_MODEL_H_
