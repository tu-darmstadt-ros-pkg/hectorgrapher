/*
 * Copyright 2021 The Cartographer Authors
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

#include <cmath>
#include <fstream>
#include <string>

#include "absl/strings/str_split.h"
#include "cartographer/common/port.h"
#include "cartographer/ground_truth/autogenerate_ground_truth.h"
#include "cartographer/ground_truth/proto/relations.pb.h"
#include "cartographer/ground_truth/relations_text_file.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(csv_filename, "", "CSV-file containing the tracking data.");
DEFINE_double(pose_time_delta, 0.1,
              "Minimum duration in seconds between two poses to be added as an "
              "ground truth relation.");
DEFINE_string(output_filename, "", "File to write the ground truth proto to.");

namespace cartographer {
namespace ground_truth {
namespace {

void Run(const std::string& csv_filename, const std::string& output_filename,
         double pose_time_delta) {
  LOG(INFO) << "Autogenerating ground truth relations...";
  const proto::GroundTruth ground_truth =
      CreateRelationsFromMocapData(csv_filename, pose_time_delta);
  LOG(INFO) << "Writing " << ground_truth.relation_size() << " relations to '"
            << output_filename << "'.";
  {
    std::ofstream output_stream(output_filename,
                                std::ios_base::out | std::ios_base::binary);
    CHECK(ground_truth.SerializeToOstream(&output_stream))
        << "Could not serialize ground truth data.";
    output_stream.close();
    CHECK(output_stream) << "Could not write ground truth data.";
  }
}

}  // namespace
}  // namespace ground_truth
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program semi-automatically generates ground truth data from \n"
      "csv-files with motion capture tracking data.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_csv_filename.empty() || FLAGS_output_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "generate_ground_truth");
    return EXIT_FAILURE;
  }
  ::cartographer::ground_truth::Run(FLAGS_csv_filename, FLAGS_output_filename,
                                    FLAGS_pose_time_delta);
}
