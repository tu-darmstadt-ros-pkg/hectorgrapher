/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/ground_truth/autogenerate_ground_truth_no_lc.h"

#include <string>
#include <vector>

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace ground_truth {
namespace {

std::vector<double> ComputeCoveredDistance(
    const mapping::proto::Trajectory& trajectory) {
  std::vector<double> covered_distance;
  covered_distance.push_back(0.);
  CHECK_GT(trajectory.node_size(), 0)
      << "Trajectory does not contain any nodes.";
  for (int i = 1; i < trajectory.node_size(); ++i) {
    const auto last_pose = transform::ToRigid3(trajectory.node(i - 1).pose());
    const auto this_pose = transform::ToRigid3(trajectory.node(i).pose());
    covered_distance.push_back(
        covered_distance.back() +
        (last_pose.inverse() * this_pose).translation().norm());
  }
  return covered_distance;
}

}  // namespace

proto::GroundTruth GenerateGroundTruthNoLc(
    const mapping::proto::PoseGraph& pose_graph,
    const double min_covered_distance, const double outlier_threshold_meters,
    const double outlier_threshold_radians) {
  const mapping::proto::Trajectory& trajectory = pose_graph.trajectory(0);
  const std::vector<double> covered_distance =
      ComputeCoveredDistance(trajectory);

  int num_outliers = 0;
  proto::GroundTruth ground_truth;

  for (int i = 1; i < trajectory.node_size(); ++i) {

    const auto last_node = trajectory.node(i - 1);
    const auto this_node = trajectory.node(i);
    const auto last_pose = transform::ToRigid3(last_node.pose());
    const auto this_pose = transform::ToRigid3(this_node.pose());

    double covered_distance_in_constraint =
        std::abs(covered_distance.at(i) -
                 covered_distance.at(i - 1));

    const transform::Rigid3d expected = last_pose.inverse() * this_pose;


    auto* const new_relation = ground_truth.add_relation();
    new_relation->set_timestamp1(
        trajectory.node(i - 1).timestamp());
    new_relation->set_timestamp2(trajectory.node(i).timestamp());
    *new_relation->mutable_expected() = transform::ToProto(expected);
    new_relation->set_covered_distance(covered_distance_in_constraint);
  }
  LOG(INFO) << "Generated " << ground_truth.relation_size()
            << " relations and ignored " << num_outliers << " outliers.";
  return ground_truth;
}

}  // namespace ground_truth
}  // namespace cartographer
