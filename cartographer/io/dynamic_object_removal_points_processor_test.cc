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
DEFINE_string(gtest_color, "", "");
DEFINE_string(gtest_filter, "", "");

namespace cartographer {
namespace io {
namespace {

std::unique_ptr<PointsBatch> CreatePointsBatch1(sensor::PointCloud &static_points,
                                                Eigen::Vector3f &dynamic_point) {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back(static_points[0]);
  points_batch->points.push_back(static_points[1]);
  points_batch->points.push_back(static_points[2]);
  points_batch->points.push_back(static_points[3]);
  points_batch->points.push_back(static_points[4]);
  points_batch->points.push_back({dynamic_point}); // dynamic point in front
  return points_batch;
}

std::unique_ptr<PointsBatch> CreatePointsBatch2(sensor::PointCloud &static_points) {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back(static_points[5]);
  points_batch->points.push_back(static_points[6]);
  points_batch->points.push_back(static_points[7]);
  points_batch->points.push_back(static_points[8]);
  points_batch->points.push_back(static_points[9]);
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
              open_view_deletion = false,
              search_depth = -1
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

    static_points_.push_back({Eigen::Vector3f{-1.0f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{-0.5f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{0.0f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{0.5f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{1.0f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{-1.01f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{-0.5f, 2.01f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{0.01f, 2.01f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{0.51f, 2.0f, 0.0f}});
    static_points_.push_back({Eigen::Vector3f{1.01f, 2.0f, 0.0f}});

    do {
      pipeline.back()->Process(CreatePointsBatch1(static_points_, dynamic_point_));
      pipeline.back()->Process(CreatePointsBatch2(static_points_));
    } while (pipeline.back()->Flush() == cartographer::io::PointsProcessor::FlushResult::kRestartStream);

//    map_ = dynamic_cast<DynamicObjectsRemovalPointsProcessor*>(pipeline.back().get())->map_;
  }

  std::shared_ptr<std::vector<char>> fake_file_writer_output_ =
      std::make_shared<std::vector<char>>();
  std::unique_ptr<cartographer::common::LuaParameterDictionary>
      pipeline_dictionary_;
  sensor::PointCloud static_points_;
  Eigen::Vector3f dynamic_point_ = Eigen::Vector3f{0.0f, 1.0f, 0.0f};
  sensor::TimedPointCloud map_;
};

TEST_F(DynamicObjectRemovalPointsProcessorTest, NumberOfPointsCheck) {
  Run("test_wedge.ply");
  EXPECT_EQ(map_.size(), 10);
  // TODO(bhirschel) check that the right points are still there
}

TEST_F(DynamicObjectRemovalPointsProcessorTest, PointsValueCheck) {
  Run("test_wedge.ply");
  // Map to std vector
  std::vector<std::vector<float>> map_points_std, static_points_std;
  for (auto & map_point : map_) {
    std::vector<float> v;
    v.resize(map_point.position.size());
    Eigen::Vector3f::Map(&v[0], map_point.position.size()) = map_point.position;
    map_points_std.push_back(v);
  }
  for (auto & static_point : static_points_) {
    std::vector<float> v;
    v.resize(static_point.position.size());
    Eigen::Vector3f::Map(&v[0], static_point.position.size()) = static_point.position;

    // Check that ALL static points v are within the final map
    EXPECT_TRUE(std::find(map_points_std.begin(), map_points_std.end(), v) != map_points_std.end());
  }
  // Check that the dynamic point is NOT within the final map
  std::vector<float> v;
  v.resize(dynamic_point_.size());
  Eigen::Vector3f::Map(&v[0], dynamic_point_.size()) = dynamic_point_;
  EXPECT_TRUE(std::find(map_points_std.begin(), map_points_std.end(), v) == map_points_std.end());
}

}
}
}