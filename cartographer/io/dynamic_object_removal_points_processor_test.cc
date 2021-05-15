//
// Created by ubuntu on 13.04.21.
//

#include "cartographer/io/dynamic_object_removal_points_processor.h"

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/io/fake_file_writer.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

std::unique_ptr<PointsBatch> CreatePointsBatch1() {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back({Eigen::Vector3f{-1.0f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{-0.5f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.5f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{1.0f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 1.0f, 0.0f}}); // dynamic point in front
  return points_batch;
}

std::unique_ptr<PointsBatch> CreatePointsBatch2() {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back({Eigen::Vector3f{-1.01f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{-0.5f, 2.01f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.01f, 2.01f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.51f, 2.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{1.01f, 2.0f, 0.0f}});
  return points_batch;
}

::cartographer::io::FileWriterFactory CreateFakeFileWriterFactory(
    const std::string& expected_filename,
    std::shared_ptr<std::vector<char>> fake_file_writer_output) {
  return [&fake_file_writer_output,
      &expected_filename](const std::string& full_filename) {
    EXPECT_EQ(expected_filename, full_filename);
    return ::absl::make_unique<::cartographer::io::FakeFileWriter>(
        full_filename, fake_file_writer_output);
  };
}

std::vector<std::unique_ptr<::cartographer::io::PointsProcessor>>
CreatePipelineFromDictionary(
    common::LuaParameterDictionary* const pipeline_dictionary,
    ::cartographer::io::FileWriterFactory file_writer_factory) {
  auto builder =
      ::absl::make_unique<::cartographer::io::PointsProcessorPipelineBuilder>();
  builder->Register(
      DynamicObjectsRemovalPointsProcessor::kConfigurationFileActionName,
      [file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return DynamicObjectsRemovalPointsProcessor::FromDictionary(
            file_writer_factory, dictionary, next);
      });

  return builder->CreatePipeline(pipeline_dictionary);
}

std::unique_ptr<common::LuaParameterDictionary> CreateParameterDictionary() {
  auto parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          R"text(
          pipeline = {
            {
              action = "dynamic_objects_removal_filter",
              filename = "test_wedge.ply",
              r_segments = 150,
              theta_segments = 63,
              phi_segments = 127,
              sensor_range_limit = 5,
              end_of_file = 445,
              probability_reduction_factor = 0.1,
              dynamic_object_probability_threshold = 1.0,
              open_view_deletion = false
            }
          }
          return pipeline
    )text",
          absl::make_unique<cartographer::common::DummyFileResolver>());
  return parameter_dictionary;
}

class DynamicObjectRemovalPointsProcessorTest : public ::testing::Test {
 protected:
  DynamicObjectRemovalPointsProcessorTest()
      : pipeline_dictionary_(CreateParameterDictionary()) {}

  void Run(const std::string& expected_filename) {
    const auto pipeline = CreatePipelineFromDictionary(
        pipeline_dictionary_.get(),
        CreateFakeFileWriterFactory(expected_filename,
                                    fake_file_writer_output_));
    EXPECT_TRUE(pipeline.size() > 0);

    do {
      pipeline.back()->Process(CreatePointsBatch1());
      pipeline.back()->Process(CreatePointsBatch2());
    } while (pipeline.back()->Flush() == cartographer::io::PointsProcessor::FlushResult::kRestartStream);

    map_ = dynamic_cast<DynamicObjectsRemovalPointsProcessor*>(pipeline.back().get())->map_;
  }

  std::shared_ptr<std::vector<char>> fake_file_writer_output_ =
      std::make_shared<std::vector<char>>();
  std::unique_ptr<cartographer::common::LuaParameterDictionary>
      pipeline_dictionary_;
  sensor::TimedPointCloud map_;
};

TEST_F(DynamicObjectRemovalPointsProcessorTest, DynamicObjectRemoved) {
  Run("test_wedge.ply");
  EXPECT_EQ(map_.size(), 10);
  // TODO(bhirschel) check that the right points are still there
  //EXPECT_FLOAT_EQ(map_)
}

}
}
}