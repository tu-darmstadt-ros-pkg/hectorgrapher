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

#include "cartographer/transform/transform_interpolation_buffer.h"

#include <algorithm>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace transform {

TransformInterpolationBuffer::TransformInterpolationBuffer(
    const mapping::proto::Trajectory& trajectory) {
  for (const mapping::proto::Trajectory::Node& node : trajectory.node()) {
    Push(common::FromUniversal(node.timestamp()),
         transform::ToRigid3(node.pose()));
  }
}

void TransformInterpolationBuffer::Push(const common::Time time,
                                        const transform::Rigid3d& transform) {
  if (!timestamped_transforms_.empty()) {
    CHECK_GE(time, latest_time()) << "New transform is older than latest.";
  }
  timestamped_transforms_.push_back(TimestampedTransform{time, transform});
}

bool TransformInterpolationBuffer::Has(const common::Time time) const {
  if (timestamped_transforms_.empty()) {
    return false;
  }
  return earliest_time() <= time && time <= latest_time();
}

transform::Rigid3d TransformInterpolationBuffer::Lookup(
    const common::Time time) const {
  CHECK(Has(time)) << "Missing transform for: " << time;
  const auto end = std::lower_bound(
      timestamped_transforms_.begin(), timestamped_transforms_.end(), time,
      [](const TimestampedTransform& timestamped_transform,
         const common::Time time) {
        return timestamped_transform.time < time;
      });
  if (end->time == time) {
    return end->transform;
  }
  const auto start = std::prev(end);
  return Interpolate(*start, *end, time).transform;
}

common::Time TransformInterpolationBuffer::LookupUntilDelta(
    common::Time start_time, double max_translation, double max_rotation,
    double max_duration, double& translation_ratio, double& rotation_ratio,
    double& time_ratio) const {
  CHECK(Has(start_time)) << "Missing transform for: " << start_time;
  auto candidate =
      std::lower_bound(timestamped_transforms_.begin(),
                       timestamped_transforms_.end(), start_time,
                       [](const TimestampedTransform& timestamped_transform,
                          const common::Time time) {
                         return timestamped_transform.time < time;
                       });
  const auto start =
      candidate->time == start_time ? candidate : std::prev(candidate);
  transform::Rigid3d start_transform =
      candidate->time == start_time
          ? start->transform
          : Interpolate(*start, *candidate, start_time).transform;

  double target_ratio = 1.0;
  while (std::next(candidate) != timestamped_transforms_.end()) {
    const transform::Rigid3d delta_pose =
        start_transform.inverse() * std::next(candidate)->transform;
    double translation_distance = std::abs(delta_pose.translation().norm());
    double rotation_distance = std::abs(
        delta_pose.rotation().angularDistance(Eigen::Quaterniond::Identity()));
    double delta_time =
        std::abs(common::ToSeconds(std::next(candidate)->time - start_time));
    ++candidate;
    translation_ratio = translation_distance / max_translation;
    rotation_ratio = rotation_distance / max_rotation;
    time_ratio = delta_time / max_duration;
    target_ratio =
        std::max(translation_ratio, std::max(rotation_ratio, time_ratio));
    if (target_ratio >= 1.0) {
      //            LOG(INFO) <<"delta "<<delta_time/target_ratio<< "\t dT " <<
      //            translation_ratio
      //                      << "\t dr " << rotation_ratio
      //                      << "\t dt " << time_ratio;
      break;
    }
  }
  common::Duration delta_duration = candidate->time - start_time;
  common::Duration corrected_delta =
      target_ratio > 1.0
          ? common::FromSeconds(
                common::ToSeconds(candidate->time - start_time) / target_ratio)
          : delta_duration;
  if (target_ratio > 1.0) {
    translation_ratio /= target_ratio;
    rotation_ratio /= target_ratio;
    time_ratio /= target_ratio;
  }
  //  LOG(INFO)<<"delta "<<common::ToSeconds(corrected_delta);
  return start_time + corrected_delta;
}

common::Time TransformInterpolationBuffer::earliest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.front().time;
}

common::Time TransformInterpolationBuffer::latest_time() const {
  CHECK(!empty()) << "Empty buffer.";
  return timestamped_transforms_.back().time;
}

bool TransformInterpolationBuffer::empty() const {
  return timestamped_transforms_.empty();
}


void TransformInterpolationBuffer::DeleteUntil(common::Time time) {
  while ((!timestamped_transforms_.empty()) && timestamped_transforms_.front().time < time) {
    timestamped_transforms_.pop_front();
  }
}

}  // namespace transform
}  // namespace cartographer
