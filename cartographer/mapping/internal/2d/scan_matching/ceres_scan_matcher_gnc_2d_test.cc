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

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_gnc_2d.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

#include <memory>
#include <mapping/2d/tsdf_2d.h>

#include "absl/memory/memory.h"
//#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {


class CeresScanMatcherGncTest : public ::testing::Test {
protected:
    CeresScanMatcherGncTest() :
      tsdf_(MapLimits(0.1, Eigen::Vector2d(2.05, 2.05), CellLimits(40, 40)),
            0.3, 1.0, &conversion_tables_) {
      auto parameter_dictionary = common::MakeDictionary(
          "return { "
          "truncation_distance = 0.07,"
          "maximum_weight = 128,"
          "update_free_space = true,"
          "normal_estimation_options = {"
          "use_pca = false,"
          "const_weight = 0.1,"
          "tsdf_weight_scale = 0.0,"
          "sort_range_data = true,"
          "num_normal_samples = 1,"
          "sample_radius = 0.15,"
          "},"
          "min_normal_weight = 0.1,"
          "free_space_weight = 0.5,"
          "project_sdf_distance_to_scan_normal = true,"
          "update_weight_range_exponent = 0,"
          "update_weight_angle_scan_normal_to_ray_kernel_bandwith = 0,"
          "update_weight_distance_cell_to_hit_kernel_bandwith = 10,"
          "}");
      options_ = CreateTSDFRangeDataInserterOptions2D(
          parameter_dictionary.get());
      range_data_inserter_ = absl::make_unique<TSDFRangeDataInserter2D>(
          options_);

      point_cloud_.push_back({Eigen::Vector3f{0.f, 0.5, 0.f}});
      sensor::RangeData range_data;
      for (float x = -0.005; x <= 0.005f; x += 0.005) {
        range_data.returns.push_back({Eigen::Vector3f{x, 1.0f, 0.f}});
      }
      range_data.origin.x() = -0.5f;
      range_data.origin.y() = -0.5f;
      range_data_inserter_->Insert(range_data, &tsdf_);
      tsdf_.FinishUpdate();

      auto parameter_dictionary_matcher = common::MakeDictionary(R"text(
          return {
            occupied_space_weight = 0.25,
            translation_weight = 0.1,
            rotation_weight = 0.001,
            empty_space_cost = 0.5,
            ceres_solver_options = {
              use_nonmonotonic_steps = true,
              max_num_iterations = 20,
              num_threads = 1,
            },
          })text");
      const proto::CeresScanMatcherOptions2D options =
          CreateCeresScanMatcherOptions2D(parameter_dictionary_matcher.get());
      ceres_scan_matcher_ = absl::make_unique<CeresScanMatcher2D>(options);
    }

  void TestFromInitialPose(const transform::Rigid2d& initial_pose) {
    transform::Rigid2d pose;
    const transform::Rigid2d expected_pose =
        transform::Rigid2d::Translation({-0.0, 0.5});
    ceres::Solver::Summary summary;
//    unsigned long new_size = point_cloud_.size();
//    ceres_scan_matcher_->newMatch(new_size);
    ceres_scan_matcher_->Match(initial_pose.translation(), initial_pose,
                               point_cloud_, tsdf_, &pose,
                               &summary);
    EXPECT_NEAR(0., summary.final_cost, 1e-2) << summary.FullReport();
    EXPECT_THAT(pose, transform::IsNearly(expected_pose, 1e-2))
        << "Actual: " << transform::ToProto(pose).DebugString()
        << "\nExpected: " << transform::ToProto(expected_pose).DebugString();
  }

  ValueConversionTables conversion_tables_;
  ::cartographer::mapping::proto::TSDFRangeDataInserterOptions2D options_;
//  ProbabilityGrid probability_grid_;
  TSDF2D tsdf_;
  std::unique_ptr<TSDFRangeDataInserter2D> range_data_inserter_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
};

TEST_F(CeresScanMatcherGncTest, testPerfectEstimate)
{
  TestFromInitialPose(transform::Rigid2d::Translation({-0.5, 0.5}));
}

TEST_F(CeresScanMatcherGncTest, testOptimizeAlongX) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.3, 0.5}));
}

TEST_F(CeresScanMatcherGncTest, testOptimizeAlongY) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.45, 0.3}));
}

TEST_F(CeresScanMatcherGncTest, testOptimizeAlongXY) {
  TestFromInitialPose(transform::Rigid2d::Translation({-0.3, 0.3}));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}