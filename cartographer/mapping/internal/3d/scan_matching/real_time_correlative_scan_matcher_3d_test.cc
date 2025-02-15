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

#include "cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"

#include <memory>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

class RealTimeCorrelativeScanMatcher3DTest : public ::testing::Test {
 protected:
  RealTimeCorrelativeScanMatcher3DTest()
      : hybrid_grid_(0.1f, &conversion_tables_),
        expected_pose_(Eigen::Vector3d(-1., 0., 0.),
                       Eigen::Quaterniond::Identity()) {
    for (const Eigen::Vector3f& point :
         {Eigen::Vector3f(-3.f, 2.f, 0.f), Eigen::Vector3f(-4.f, 2.f, 0.f),
          Eigen::Vector3f(-5.f, 2.f, 0.f), Eigen::Vector3f(-6.f, 2.f, 0.f),
          Eigen::Vector3f(-6.f, 3.f, 1.f), Eigen::Vector3f(-6.f, 4.f, 2.f),
          Eigen::Vector3f(-7.f, 3.f, 1.f)}) {
      point_cloud_.push_back({point});
      hybrid_grid_.SetProbability(
          hybrid_grid_.GetCellIndex(expected_pose_.cast<float>() * point), 1.);
    }

    auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          linear_search_window = 0.3,
          angular_search_window = math.rad(1.),
          translation_delta_cost_weight = 1e-1,
          rotation_delta_cost_weight = 1.,
        })text");
    real_time_correlative_scan_matcher_.reset(
        new RealTimeCorrelativeScanMatcher3D(
            CreateRealTimeCorrelativeScanMatcherOptions(
                parameter_dictionary.get())));
  }

  void TestFromInitialPose(const transform::Rigid3d& initial_pose) {
    transform::Rigid3d pose;

    const float score = real_time_correlative_scan_matcher_->Match(
        initial_pose, point_cloud_, hybrid_grid_, &pose);
    LOG(INFO) << "Score: " << score;
    EXPECT_THAT(pose, transform::IsNearly(expected_pose_, 1e-3));
  }

  ValueConversionTables conversion_tables_;
  HybridGrid hybrid_grid_;
  transform::Rigid3d expected_pose_;
  sensor::PointCloud point_cloud_;
  std::unique_ptr<RealTimeCorrelativeScanMatcher3D>
      real_time_correlative_scan_matcher_;
};

TEST_F(RealTimeCorrelativeScanMatcher3DTest, PerfectEstimate) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.)));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, AlongX) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.8, 0., 0.)));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, AlongZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., -0.2)));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, AlongXYZ) {
  TestFromInitialPose(
      transform::Rigid3d::Translation(Eigen::Vector3d(-0.9, -0.2, 0.2)));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, RotationAroundX) {
  TestFromInitialPose(transform::Rigid3d(
      Eigen::Vector3d(-1., 0., 0.),
      Eigen::AngleAxisd(0.8 / 180. * M_PI, Eigen::Vector3d(1., 0., 0.))));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, RotationAroundY) {
  TestFromInitialPose(transform::Rigid3d(
      Eigen::Vector3d(-1., 0., 0.),
      Eigen::AngleAxisd(0.8 / 180. * M_PI, Eigen::Vector3d(0., 1., 0.))));
}

TEST_F(RealTimeCorrelativeScanMatcher3DTest, RotationAroundYZ) {
  TestFromInitialPose(transform::Rigid3d(
      Eigen::Vector3d(-1., 0., 0.),
      Eigen::AngleAxisd(0.8 / 180. * M_PI, Eigen::Vector3d(0., 1., 1.))));
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
