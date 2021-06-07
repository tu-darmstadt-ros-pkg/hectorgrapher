//=================================================================================================
// Copyright (c) 2021, Kevin Daun, TU Darmstadt
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

#ifndef CARTOGRAPHER_MAPPING_3D_DEBUG_LOGGER_H_
#define CARTOGRAPHER_MAPPING_3D_DEBUG_LOGGER_H_

#include <fstream>
#include <iostream>
#include <string>

#include "cartographer/mapping/internal/3d/state.h"

namespace cartographer {
namespace mapping {

class DebugLogger {
 public:
  explicit DebugLogger(const std::string& filename)
      : initialization_time_(common::FromUniversal(0)) {
    file_.open(filename);
    file_ << "t_universal,t_rel,p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,roll,"
             "pitch,yaw,translation_ratio,rotation_ratio,time_ratio\n";
  }

  void AddEntry(const ControlPoint& c) {
    if (initialization_time_ == common::FromUniversal(0)) {
      initialization_time_ = c.time;
    }
    const transform::Rigid3d rotation_c(
        Eigen::Map<const Eigen::Vector3d>(c.state.translation.data()),
        Eigen::Quaternion<double>(c.state.rotation[0], c.state.rotation[1],
                                  c.state.rotation[2], c.state.rotation[3]));
    file_ << common::ToUniversal(c.time) << ","
          << common::ToSeconds(c.time - initialization_time_) << ","
          << c.state.translation[0] << "," << c.state.translation[1] << ","
          << c.state.translation[2] << "," << c.state.velocity[0] << ","
          << c.state.velocity[1] << "," << c.state.velocity[2] << ","
          << c.state.rotation[0] << "," << c.state.rotation[1] << ","
          << c.state.rotation[2] << "," << c.state.rotation[3] << ","
          << transform::GetRoll(rotation_c) << ","
          << transform::GetPitch(rotation_c) << ","
          << transform::GetYaw(rotation_c) << "," << c.translation_ratio << ","
          << c.rotation_ratio << "," << c.time_ratio << "\n";
  }

  ~DebugLogger() { file_.close(); }

  common::Time initialization_time_;
  std::ofstream file_;
};
}  // namespace mapping
}  // namespace cartographer
#endif  // CARTOGRAPHER_MAPPING_3D_DEBUG_LOGGER_H_