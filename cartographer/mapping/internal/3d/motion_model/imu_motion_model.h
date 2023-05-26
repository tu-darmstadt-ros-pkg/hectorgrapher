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

#ifndef IMU_MOTION_MODEL_H_
#define IMU_MOTION_MODEL_H_

#include "cartographer/mapping/internal/3d/motion_model/motion_model.h"

namespace cartographer {
namespace mapping {

class IMUMotionModel : public MotionModel {
 public:
  State initialize(const common::Time& time) override;
  State ExtrapolateState(const common::Time& time) const override;
  bool HasDataUntil(const common::Time& time) const override;
  void UpdateState(const State& state, const common::Time& time) override;
  bool HasDataBefore(const common::Time& time) const override;
  transform::Rigid3d RelativeTransform(const common::Time& t0,
                                       const common::Time& t1) override;

 private:
  State stateAt(const common::Time& time, bool use_imu_data) const;
  // buffer data
  int buffer_t0_idx = -1;
  int buffer_t1_idx = -1;
  common::Time buffer_t0_time;
  common::Time buffer_t1_time;
  Eigen::Quaterniond buffer_rotation_t0;
  Eigen::Quaterniond buffer_rotation_t1;
  int num_hits = 0;
  int num_continues = 0;
  int num_research = 0;
  int num_no_buffer = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // IMU_MOTION_MODEL_H_
