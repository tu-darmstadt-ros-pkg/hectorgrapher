/*
 * Copyright 2019 Karim Barth
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

#include "cartographer/sensor/internal/normal_estimation_helper.h"

#include <cmath>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::ContainerEq;

TEST(NormalEstimationHelperTest, DifferentNormalOrientation) {

  std::vector<Eigen::Vector2f> normal_list = {Eigen::Vector2f{1.f, 0.f}, Eigen::Vector2f{1.f, 1.f},
                                             Eigen::Vector2f{0.f, 1.f}, Eigen::Vector2f{-1.f, 1.f},
                                             Eigen::Vector2f{-1.f, 0.f}, Eigen::Vector2f{-1.f, -1.f},
                                             Eigen::Vector2f{0.f, -1.f}, Eigen::Vector2f{1.f, -1.f}};

  std::vector<float> orientation_list = {0., M_PI_4, M_PI_2, 3* M_PI_4, M_PI, 5*M_PI_4, 6* M_PI_4, 7*M_PI_4};

  std::vector<float> res(orientation_list.size());

  std::transform(normal_list.begin(), normal_list.end(), res.begin(), [](Eigen::Vector2f vec){
    return NormalOrientation(vec);
  } );

  ASSERT_THAT(res, ContainerEq(orientation_list));
}

TEST(NormalEstimationHelperTest, SimplePca) {

 PointCloud point_cloud = {{Eigen::Vector3f{-2.f, 2.01f, 0.f}},
                           {Eigen::Vector3f{-1.f, 2.015f, 0.f}},
                           {Eigen::Vector3f{0.f, 1.99f, 0.f}},
                           {Eigen::Vector3f{1.f, 1.999f, 0.f}},
                           {Eigen::Vector3f{2.f, 2.f, 0.f}}};

 std::set<size_t> neighbors = {0, 1, 2, 3, 4};
 auto res = Pca(neighbors, point_cloud);
 auto angle = NormalOrientation(res);
 EXPECT_NEAR(angle, M_PI_2, 0.01);


}

TEST(NormalEstimationHelperTest, SimplePCAWithMinNumberOfPoints) {

  PointCloud point_cloud = {{Eigen::Vector3f{-2.f, 2.01f, 0.f}},
                            {Eigen::Vector3f{-1.f, 2.015f, 0.f}},
                            {Eigen::Vector3f{0.f, 1.99f, 0.f}},
                            {Eigen::Vector3f{1.f, 1.999f, 0.f}},
                            {Eigen::Vector3f{2.f, 2.f, 0.f}}};

  std::set<size_t> neighbors = {1, 2, 3};
  auto res = Pca(neighbors, point_cloud);
  auto angle = NormalOrientation(res);
  EXPECT_NEAR(angle, M_PI_2, 0.01);


}

TEST(NormalEstimationHelperTest, DiagonalPCA) {

  PointCloud point_cloud = {{Eigen::Vector3f{-1.f, 3.0f, 0.f}},
                            {Eigen::Vector3f{0.f, 2.0f, 0.f}},
                            {Eigen::Vector3f{1.f, 1.f, 0.f}},
                            {Eigen::Vector3f{2.f, 0.f, 0.f}},
                            {Eigen::Vector3f{3.f, -1.f, 0.f}}};

  std::set<size_t> neighbors = {0, 1, 2, 3, 4};
  auto res = Pca(neighbors, point_cloud);
  auto angle = NormalOrientation(res);
  EXPECT_NEAR(angle, M_PI_4, 0.01);


}


TEST(NormalEstimationHelperTest, SimpleSelectNeighborhood) {
  PointCloud point_cloud = {{Eigen::Vector3f{-1.f, 3.0f, 0.f}},
                            {Eigen::Vector3f{0.f, 2.0f, 0.f}},
                            {Eigen::Vector3f{1.f, 1.f, 0.f}},
                            {Eigen::Vector3f{2.f, 0.f, 0.f}},
                            {Eigen::Vector3f{3.f, -1.f, 0.f}}};

  for(size_t i= 0; i < point_cloud.size(); ++i){
    RangefinderPoint point = point_cloud[i];
    auto res = SelectNeighborhood(point, point_cloud);
    ASSERT_TRUE(res.size() >= 3);

  }

}

TEST(NormalEstimationHelperTest, SimpleEstimateNormal) {

  PointCloud point_cloud = {{Eigen::Vector3f{-2.f, 2.f, 0.f}},
                            {Eigen::Vector3f{-1.f, 2.f, 0.f}},
                            {Eigen::Vector3f{0.f, 2.f, 0.f}},
                            {Eigen::Vector3f{1.f, 2.f, 0.f}},
                            {Eigen::Vector3f{2.f, 2.f, 0.f}}};

  for(auto vec : point_cloud){
    auto res = EstimateNormal(vec, point_cloud);
    auto angle = NormalOrientation(res);
    EXPECT_NEAR(angle, 3*M_PI_2, 0.01);
  }
}

TEST(NormalEstimationHelperTest, DiagonalEstimateNormal) {

  PointCloud point_cloud = {{Eigen::Vector3f{-1.f, 3.0f, 0.f}},
                            {Eigen::Vector3f{0.f, 2.0f, 0.f}},
                            {Eigen::Vector3f{1.f, 1.f, 0.f}},
                            {Eigen::Vector3f{2.f, 0.f, 0.f}},
                            {Eigen::Vector3f{3.f, -1.f, 0.f}}};

  for(auto vec : point_cloud){
    auto res = EstimateNormal(vec, point_cloud);
    auto angle = NormalOrientation(res);
    EXPECT_NEAR(angle, 5*M_PI_4, 0.01);
  }
}

TEST(NormalEstimationHelperTest, SimpleGenerateNormalHistogram) {

  PointCloud point_cloud = {{Eigen::Vector3f{0.f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-0.25f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-0.5f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-0.75f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-1.f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-2.f, 1.5f, 0.f}},
                            {Eigen::Vector3f{-2.f, 0.5f, 0.f}},
                            {Eigen::Vector3f{-2.f, -0.5f, 0.f}},
                            {Eigen::Vector3f{-2.f, -1.5f, 0.f}},
                            {Eigen::Vector3f{-1.f, -1.5f, 0.f}},
                            {Eigen::Vector3f{0.f, -1.5f, 0.f}},
                            {Eigen::Vector3f{1.f, -1.5f, 0.f}},
                            {Eigen::Vector3f{2.f, -1.5f, 0.f}},
                            {Eigen::Vector3f{2.f, -0.5f, 0.f}},
                            {Eigen::Vector3f{2.f, 0.5f, 0.f}},
                            {Eigen::Vector3f{2.f, 1.5f, 0.f}},
                            {Eigen::Vector3f{1.f, 1.5f, 0.f}}};

  uint histogramSize = 10;
  auto histogram = GenerateNormalHistogram( point_cloud, histogramSize);
  std::vector<int> lengths;
  lengths.resize(histogramSize);

  std::transform (histogram.begin(), histogram.end(), lengths.begin(), [](std::vector<size_t> bin){
    return bin.size();
  });

  std::cout << "Histogram size: ";
  for(int i = 0; i < lengths.size(); ++i) {
    if(i == 0) {
      std::cout << lengths[i];
    } else {
      std::cout << ", " << lengths[i];
    }
  }
  std::cout << std::endl;
  PointCloud results;
  uint points_to_remove = 9;
  std::random_device rd;
  std::mt19937 gen(rd());

  for(size_t i = 0; i < points_to_remove; ++i) {
    size_t histogram_index = std::max_element(lengths.begin(),lengths.end()) - lengths.begin();
    auto& bin = histogram[histogram_index];

    std::uniform_int_distribution<> dis(0, bin.size() - 1);
    auto point_index = dis(gen);

    lengths[histogram_index] = lengths[histogram_index] - 1;
    bin.erase(bin.begin() + point_index);
  }

  for(size_t i = 0; i < histogram.size(); ++i) {
    auto& bin = histogram[i];
    for(size_t j = 0; j < bin.size(); ++j) {
      results.push_back(point_cloud[bin[j]]);
    }
  }

  std::cout << "Histogram filtered sizes: ";
  for(int i = 0; i < lengths.size(); ++i) {
    if(i == 0) {
      std::cout << lengths[i];
    } else {
      std::cout << ", " << lengths[i];
    }
  }
  std::cout << std::endl;

  ASSERT_TRUE(point_cloud.size() - results.size() == points_to_remove);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
