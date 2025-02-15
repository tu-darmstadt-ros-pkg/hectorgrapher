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

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"

namespace cartographer {
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
  TransformInterpolationBuffer() = default;
  explicit TransformInterpolationBuffer(
      const mapping::proto::Trajectory& trajectory);

  // Adds a new transform to the buffer.
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp when either max_translation, max_rotation,
  // max_duration is reached with respect to the pose at start_time.
  common::Time LookupUntilDelta(common::Time start_time, double max_translation,
                                double max_rotation, double max_duration,
                                double& translation_ratio,
                                double& rotation_ratio,
                                double& time_ratio) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

  // Removes all observations before time from the buffer.
  void DeleteUntil(common::Time time);

 private:
  std::deque<TimestampedTransform> timestamped_transforms_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
