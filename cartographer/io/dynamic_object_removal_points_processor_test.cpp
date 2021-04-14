//
// Created by ubuntu on 13.04.21.
//

#include "cartographer/io/probability_grid_points_processor.h"

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/io/fake_file_writer.h"
#include "cartographer/io/dynamic_object_removal_points_processor.h.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

std::unique_ptr<PointsBatch> CreatePointsBatch() {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 0.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 1.0f, 2.0f}});
  points_batch->points.push_back({Eigen::Vector3f{1.0f, 2.0f, 4.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 3.0f, 5.0f}});
  points_batch->points.push_back({Eigen::Vector3f{3.0f, 0.0f, 6.0f}});
  return points_batch;
}

class DynamicObjectRemovalPointsProcessorTest : public ::testing::Test {
 protected:
  DynamicObjectRemovalPointsProcessorTest()
      : pipeline_dictionary_(CreateParameterDictionary()) {}

  void Run(const std::string& expected_filename) {
    const auto pipeline = CreatePipelineFromDictionary(
        pipeline_dictionary_.get(), dummy_trajectories_,
        CreateFakeFileWriterFactory(expected_filename,
                                    fake_file_writer_output_));
    EXPECT_TRUE(pipeline.size() > 0);

    do {
      pipeline.back()->Process(CreatePointsBatch());
    } while (pipeline.back()->Flush() ==
        cartographer::io::PointsProcessor::FlushResult::kRestartStream);
  }

  std::shared_ptr<std::vector<char>> fake_file_writer_output_ =
      std::make_shared<std::vector<char>>();
  std::unique_ptr<cartographer::common::LuaParameterDictionary>
      pipeline_dictionary_;
  const std::vector<mapping::proto::Trajectory> dummy_trajectories_;
};

TEST_F(ProbabilityGridPointsProcessorTest, WriteProto) {
  const auto expected_prob_grid_proto = CreateExpectedProbabilityGrid(
      CreatePointsBatch(),
      pipeline_dictionary_->GetArrayValuesAsDictionaries().front().get());
  Run("map.pb");
  EXPECT_THAT(*fake_file_writer_output_,
              ::testing::ContainerEq(expected_prob_grid_proto));
}

}
}
}