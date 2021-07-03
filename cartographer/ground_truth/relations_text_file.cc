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

#include "cartographer/ground_truth/relations_text_file.h"

#include <fstream>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace ground_truth {

namespace {

common::Time UnixToCommonTime(double unix_time) {
  constexpr int64 kUtsTicksPerSecond = 10000000;
  return common::FromUniversal(common::kUtsEpochOffsetFromUnixEpochInSeconds *
                               kUtsTicksPerSecond) +
         common::FromSeconds(unix_time);
}

::cartographer::common::Time FromRos(const int64& time_ns) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (::cartographer::common::
           kUtsEpochOffsetFromUnixEpochInSeconds)*10000000ll +
      (time_ns + 50) / 100);  // + 50 to get the rounding correct.
}

transform::TimestampedTransform ParseLine(const std::string& line) {
  const std::vector<std::string> split_line =
      absl::StrSplit(line, ',', absl::SkipEmpty());
  transform::TimestampedTransform result;
  result.time = FromRos(std::stoll(split_line[2]));

  Eigen::Vector3d translation;
  Eigen::Quaterniond orientation;
  translation.x() = std::stod(split_line[4]);
  translation.y() = std::stod(split_line[5]);
  translation.z() = std::stod(split_line[6]);
  orientation.x() = std::stod(split_line[7]);
  orientation.y() = std::stod(split_line[8]);
  orientation.z() = std::stod(split_line[9]);
  orientation.w() = std::stod(split_line[10]);
  //  LOG(INFO)<< translation.x() << " " << translation.y() << " " <<
  //  translation.z() << " " <<
  //    orientation.x() << " " << orientation.y() << " " << orientation.z() << "
  //    " << orientation.w();
  result.transform = transform::Rigid3d(translation, orientation);
  return result;
}

}  // namespace

proto::GroundTruth ReadRelationsTextFile(
    const std::string& relations_filename) {
  proto::GroundTruth ground_truth;
  std::ifstream relations_stream(relations_filename.c_str());
  double unix_time_1, unix_time_2, x, y, z, roll, pitch, yaw;
  while (relations_stream >> unix_time_1 >> unix_time_2 >> x >> y >> z >>
         roll >> pitch >> yaw) {
    const common::Time common_time_1 = UnixToCommonTime(unix_time_1);
    const common::Time common_time_2 = UnixToCommonTime(unix_time_2);
    const transform::Rigid3d expected =
        transform::Rigid3d(transform::Rigid3d::Vector(x, y, z),
                           transform::RollPitchYaw(roll, pitch, yaw));
    auto* const new_relation = ground_truth.add_relation();
    new_relation->set_timestamp1(common::ToUniversal(common_time_1));
    new_relation->set_timestamp2(common::ToUniversal(common_time_2));
    *new_relation->mutable_expected() = transform::ToProto(expected);
  }
  CHECK(relations_stream.eof());
  return ground_truth;
}

proto::GroundTruth CreateRelationsFromMocapData(
    const std::string& relations_filename, double time_delta) {
  std::ifstream relations_stream(relations_filename.c_str());
  common::Time header_time;
  std::string frame_id;

  std::ifstream csv_file(relations_filename);
  std::vector<std::vector<std::string> > data_list;
  std::string line;
  getline(csv_file, line);
  std::vector<transform::TimestampedTransform> transforms;
  while (getline(csv_file, line)) {
    transforms.push_back(ParseLine(line));
  }

  proto::GroundTruth ground_truth;
  transform::TimestampedTransform last_transform = transforms.front();
  Eigen::Vector3d translation = {0.07, 0.05, -0.013};
  Eigen::Quaterniond orientation =
      transform::RollPitchYaw(3.1416, 0.0, -1.5708);
  transform::Rigid3d pose_correction =
      transform::Rigid3d(translation, orientation);
  for (const transform::TimestampedTransform& current_transform : transforms) {
    LOG(INFO) << "aa";
    double dt = common::ToSeconds(current_transform.time - last_transform.time);
    if (dt >= time_delta) {
      auto* const new_relation = ground_truth.add_relation();
      new_relation->set_timestamp1(common::ToUniversal(last_transform.time));
      new_relation->set_timestamp2(common::ToUniversal(current_transform.time));
      *new_relation->mutable_expected() = transform::ToProto(
          (last_transform.transform * pose_correction).inverse() *
          (current_transform.transform * pose_correction));
      last_transform = current_transform;
    }
  }
  return ground_truth;
}

}  // namespace ground_truth
}  // namespace cartographer
