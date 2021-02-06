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

#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
namespace cartographer {
namespace mapping {
namespace {

float NormalTo2DAngle(const Eigen::Vector3f& v) {
  return std::atan2(v[1], v[0]);
}

float Normal2DTo2DAngle(const Eigen::Vector2f& v) {
  return std::atan2(v[1], v[0]);
}

// calculates the distance of x from the line passing through p1 -> p2
float calcDistanceFromLine(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
                           const Eigen::Vector3f& x) {
  float d = abs( (p2.x() - p1.x()) * (p1.y() - x.y()) -
                (p1.x() - x.x()) * (p2.y() - p1.y()) ) / (p2 - p1).norm();
  return abs(d);
}

// Estimate the normal of an estimation_point as the arithmetic mean of the the
// normals of the vectors from estimation_point to each point in the
// sample_window.
std::pair<float, float> EstimateNormal(const sensor::PointCloud& returns,
                                       const size_t estimation_point_index,
                                       const size_t sample_window_begin,
                                       const size_t sample_window_end,
                                       const Eigen::Vector3f& sensor_origin,
                                       const double sample_radius) {
    bool pr = sample_window_end < sample_window_begin;
    const Eigen::Vector3f &estimation_point = returns[estimation_point_index].position;
    const int num_returns = returns.size();
    int num_samples = 0;
    if ((sample_window_end - sample_window_begin + num_returns) % num_returns <
        2) {
      return std::make_pair<float, float>(
          NormalTo2DAngle(sensor_origin - estimation_point),
          static_cast<float>(num_samples));
    }
    Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
    const Eigen::Vector3f &estimation_point_to_observation =
        sensor_origin - estimation_point;
    bool use_ransac = true;  // TODO(mtorchalla) add proto option
    int max_ransac_iterations = 60;  // TODO(mtorchalla) add proto option
    Eigen::Vector3f best_fit_normal;
    float ransac_threshold = sample_radius * 0.33;
    float best_fit_score = INFINITY;
    int best_fit_outlier = INFINITY;
    int best_fit_inlier = 0;
    for (size_t sample_point_index_offset = 0;
         (sample_window_begin + sample_point_index_offset) % num_returns !=
         sample_window_end; ++sample_point_index_offset) {
      int sample_point_index =
          (sample_window_begin + sample_point_index_offset) % num_returns;
      if (sample_point_index == estimation_point_index) continue;
      const Eigen::Vector3f &sample_point = returns[sample_point_index].position;
      const Eigen::Vector3f &tangent = estimation_point - sample_point;
      Eigen::Vector3f sample_normal = {-tangent[1], tangent[0], 0.f};
      constexpr float kMinNormalLength = 1e-6f;
      if (sample_normal.norm() < kMinNormalLength) {
        continue;
      }
      // Ensure sample_normal points towards 'sensor_origin'.
      if (sample_normal.dot(estimation_point_to_observation) < 0) {
        sample_normal = -sample_normal;
      }
      sample_normal.normalize();

      mean_normal += sample_normal;
      num_samples++;
    }
//  LOG(INFO) << "Num outlier: " << best_fit_outlier;

    // "RANSAC"
    if (use_ransac) {
      std::default_random_engine generator;
      std::uniform_int_distribution<int> distribution(0,
                                                      (int)sample_window_end - 1);
      for (int iteration = 0; iteration < max_ransac_iterations; iteration++) {
        float residual = 0;
        int num_outlier = 0;
        int p1_index = distribution(generator);
        p1_index = (sample_window_begin + p1_index) % num_returns;
        int p2_index = distribution(generator);
        while (p1_index == p2_index)
          p2_index = distribution(generator);
        p2_index = (sample_window_begin + p2_index) % num_returns;
        auto p1 = returns[p1_index].position;
        auto p2 = returns[p2_index].position;
        // Calculate outliers:
        for (size_t sample_point_index_offset_2 = 0;
             (sample_window_begin + sample_point_index_offset_2) %
             num_returns !=
             sample_window_end; ++sample_point_index_offset_2) {
          int sample_point_index_2 =
              (sample_window_begin + sample_point_index_offset_2) %
              num_returns;
          if (sample_point_index_2 == p1_index ||
              sample_point_index_2 == p2_index)
            continue;
          auto x = returns[sample_point_index_2].position;
          float res_i = calcDistanceFromLine(p1, p2, x);
          if (res_i > ransac_threshold)
            ++num_outlier;
          residual += res_i;
        }
        if (residual <= best_fit_score && num_outlier < best_fit_outlier) {
          best_fit_outlier = num_outlier;
          best_fit_score = residual;
//            LOG(INFO) << "P1: " << p1.x() << "," << p1.y();
//            LOG(INFO) << "P2: " << p2.x() << "," << p2.y();
          const Eigen::Vector3f &tangent = p1 - p2;
          best_fit_normal.x() = -tangent[1];
          best_fit_normal.y() = tangent[0];
//            LOG(INFO) << "TANGENT: " << tangent.x() << "," << tangent.y();
//            LOG(INFO) << "BEST NORMAL: " << best_fit_normal.x() << "," << best_fit_normal.y();
        }
      }
// //     OLD:
//          float residual = 0;
//          int num_outlier = 0;
//          for (size_t sample_point_index_offset_2 = 0;
//               (sample_window_begin + sample_point_index_offset_2) %
//               num_returns !=
//               sample_window_end; ++sample_point_index_offset_2) {
//            int sample_point_index_2 =
//                (sample_window_begin + sample_point_index_offset_2) %
//                num_returns;
//            if (sample_point_index_2 == estimation_point_index ||
//                sample_point_index_2 == sample_point_index)
//              continue;
//            auto x = returns[sample_point_index_2].position;
//            float res_i = calcDistanceFromLine(estimation_point, sample_point,
//                                               x);
//            if (res_i > sample_radius * 0.5)
//              ++num_outlier;
//            residual += res_i;
//          }
//          if (residual < best_fit_score) {
//            best_fit_outlier = num_outlier;
//            best_fit_normal = sample_normal;
//          }
//        }
    }
    if (best_fit_normal.dot(estimation_point_to_observation) < 0) {
      best_fit_normal = -best_fit_normal;
//        LOG(INFO) << "BEST NORMAL: " << best_fit_normal.x() << "," << best_fit_normal.y();
    }
//      best_fit_normal.normalize();
//      LOG(INFO) << "BEST NORMAL: " << best_fit_normal.x() << "," << best_fit_normal.y();


    // calculate residual for mean_normal
  if (use_ransac) {
    auto p1 = estimation_point;
    float res_mean_normal = 0;
    float num_outlier_normal = 0;
    Eigen::Vector3f mean_tangent = {mean_normal[1], -mean_normal[0], 0.f};
    auto p2 = estimation_point + mean_tangent;
    for (size_t sample_point_index_offset_2 = 0;
         (sample_window_begin + sample_point_index_offset_2) %
         num_returns != sample_window_end; ++sample_point_index_offset_2) {
      int sample_point_index_2 =
          (sample_window_begin + sample_point_index_offset_2) % num_returns;
      if (sample_point_index_2 == estimation_point_index)
        continue;
      auto x = returns[sample_point_index_2].position;
      float res_i = calcDistanceFromLine(p1, p2, x);
      if (res_i > ransac_threshold)
        ++num_outlier_normal;
      res_mean_normal += res_i;
    }

//    // PRINTS:
//    LOG(INFO) << "Normal estimation for Point : " << estimation_point.x() << ", " << estimation_point.y();
//    LOG(INFO) << "Normal estimation for Origin: " << sensor_origin.x() << ", " << sensor_origin.y();
//    LOG(INFO) << "Normal averaged: " << mean_normal.x() << ", " << mean_normal.y();
//    LOG(INFO) << "Normal RANSAC  : " << best_fit_normal.x() << ", " << best_fit_normal.y();
//    LOG(INFO) << "RANSAC: Num Outlier: " << best_fit_outlier << ", Score: " << best_fit_score;
//    LOG(INFO) << "NORMAL: Num Outlier: " << num_outlier_normal << ", Score: " << res_mean_normal;
//    LOG(INFO) << "All points:";
//    for (size_t sample_point_index_offset_2 = 0;
//         (sample_window_begin + sample_point_index_offset_2) %
//         num_returns !=
//         sample_window_end; ++sample_point_index_offset_2) {
//      LOG(INFO) << "Point: " << returns[sample_point_index_offset_2].position.x() << ", " << returns[sample_point_index_offset_2].position.y();
//    }
//    LOG(INFO) << "\n";

//    LOG(INFO) << "STATS RANSAC : " << stats_ransac;
//    LOG(INFO) << "STATS AVERAGE: " << stats_avg;
    // X% outlier and better score than mean normal
    if (best_fit_outlier > num_samples * 0.05 &&
        res_mean_normal > best_fit_score) {
//      ++stats_ransac;
      return std::make_pair<float, float>(NormalTo2DAngle(best_fit_normal),
                                          static_cast<float>(num_samples));
    } else {
//      ++stats_avg;
      return std::make_pair<float, float>(NormalTo2DAngle(mean_normal),
                                          static_cast<float>(num_samples));
    }
  } else {
    return std::make_pair<float, float>(NormalTo2DAngle(mean_normal),
                                        static_cast<float>(num_samples));
  }
}

float EstimateNormalPCA(const sensor::PointCloud& returns,
                        const size_t estimation_point_index,
                        const size_t sample_window_begin,
                        const size_t sample_window_end,
                        const Eigen::Vector3f& sensor_origin) {
  const Eigen::Vector3f& estimation_point =
      returns[estimation_point_index].position;
  if (sample_window_end - sample_window_begin < 2) {
    return NormalTo2DAngle(sensor_origin - estimation_point);
  }
  Eigen::MatrixXf initial_points =
      Eigen::MatrixXf(2, sample_window_end - sample_window_begin);
  for (size_t sample_point_index = sample_window_begin;
       sample_point_index < sample_window_end; ++sample_point_index) {
    const Eigen::Vector3f& sample_point = returns[sample_point_index].position;
    initial_points.col(sample_point_index - sample_window_begin) =
        sample_point.head<2>();
  }
  Eigen::Vector2f centroid = initial_points.rowwise().mean();
  initial_points.colwise() -= centroid;
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(initial_points, Eigen::ComputeFullU);
  Eigen::Vector2f normal = svd.matrixU().col(1);
  const Eigen::Vector3f& estimation_point_to_observation =
      sensor_origin - estimation_point;
  if (normal.dot(estimation_point_to_observation.head<2>()) < 0) {
    normal = -normal;
  }
  return Normal2DTo2DAngle(normal);
}
}  // namespace

proto::NormalEstimationOptions2D CreateNormalEstimationOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::NormalEstimationOptions2D options;
  options.set_num_normal_samples(
      parameter_dictionary->GetInt("num_normal_samples"));
  options.set_sample_radius(parameter_dictionary->GetDouble("sample_radius"));
  options.set_use_pca(parameter_dictionary->GetBool("use_pca"));
  options.set_const_weight(parameter_dictionary->GetDouble("const_weight"));
  options.set_tsdf_weight_scale(parameter_dictionary->GetDouble("tsdf_weight_scale"));
  options.set_sort_range_data(parameter_dictionary->GetBool("sort_range_data"));
  CHECK_GT(options.num_normal_samples(), 0);
  CHECK_GT(options.sample_radius(), 0.0);
  return options;
}

// Estimates the normal for each 'return' in 'range_data'.
// Assumes the angles in the range data returns are sorted with respect to
// the orientation of the vector from 'origin' to 'return'.
std::vector<std::pair<float, float>> EstimateNormals(
    const sensor::RangeData& sorted_range_data,
    const proto::NormalEstimationOptions2D& normal_estimation_options) {
  std::vector<std::pair<float, float>> normals;
  normals.reserve(sorted_range_data.returns.size());
  const size_t max_num_samples = normal_estimation_options.num_normal_samples();
  const float sample_radius = normal_estimation_options.sample_radius();
  for (size_t current_point = 0;
       current_point < sorted_range_data.returns.size(); ++current_point) {
    const Eigen::Vector3f& hit =
        sorted_range_data.returns[current_point].position;
    const int num_returns = sorted_range_data.returns.size();
//    LOG(INFO) << "Sample Radius: " << sample_radius;
    int sample_window_begin_offset = 0;
    for (; (hit -
            sorted_range_data
                .returns[(current_point + sample_window_begin_offset - 1 +
                          num_returns) %
                         num_returns]
                .position)
               .norm() < sample_radius;
         --sample_window_begin_offset) {
//      LOG(INFO) << "Norm: " << (hit - sorted_range_data.returns[(current_point + sample_window_begin_offset) % num_returns].position).norm();
//      if (abs(sample_window_begin_offset) >= num_returns)
//        break;
    }

    int sample_window_end_offset = 0;
    for (;
         (hit -
          sorted_range_data
              .returns[(current_point + sample_window_end_offset) % num_returns]
              .position)
             .norm() < sample_radius;
         ++sample_window_end_offset) {
//      LOG(INFO) << "Norm: " << (hit - sorted_range_data.returns[(current_point + sample_window_end_offset) % num_returns].position).norm();
//      if (abs(sample_window_end_offset) >= num_returns)
//        break;
    }


    int sample_window_begin =
        (current_point + sample_window_begin_offset + num_returns) %
        num_returns;
    int sample_window_end =
        (current_point + sample_window_end_offset) % num_returns;

    if (normal_estimation_options.use_pca()) {
      //      normals.push_back(EstimateNormalPCA(
      //          sorted_range_data.returns, current_point, sample_window_begin,
      //          sample_window_end, sorted_range_data.origin));
    } else {
      normals.push_back(EstimateNormal(sorted_range_data.returns, current_point,
                                       sample_window_begin, sample_window_end,
                                       sorted_range_data.origin, sample_radius));
    }
  }
  return normals;
}

std::vector<std::pair<float, float>> EstimateNormalsFromTSDF(
    const sensor::RangeData& range_data, const mapping::TSDF2D& tsdf) {
  std::vector<std::pair<float, float>> normals;
  normals.reserve(range_data.returns.size());
  for (size_t current_point_idx = 0;
       current_point_idx < range_data.returns.size(); ++current_point_idx) {
    const Eigen::Vector3f& hit = range_data.returns[current_point_idx].position;
    // limits.GetCellIndex(Vector2f(-0.5f, 0.5f)
    // tsdf.GetTSD(tsdf.limits().GetCellIndex(hit.head<2>()));
    Eigen::Vector2f center =
        tsdf.limits().GetCellCenter(tsdf.limits().GetCellIndex(hit.head<2>()));
    float x0 =
        center[0] < hit[0] ? center[0] : center[0] - tsdf.limits().resolution();
    float y0 =
        center[1] < hit[1] ? center[1] : center[1] - tsdf.limits().resolution();
    float x1 = x0 + tsdf.limits().resolution();
    float y1 = y0 + tsdf.limits().resolution();

    float m00 = tsdf.GetTSD(tsdf.limits().GetCellIndex({x0, y0}));
    float m10 = tsdf.GetTSD(tsdf.limits().GetCellIndex({x1, y0}));
    float m01 = tsdf.GetTSD(tsdf.limits().GetCellIndex({x0, y1}));
    float m11 = tsdf.GetTSD(tsdf.limits().GetCellIndex({x1, y1}));

    float w00 = tsdf.GetWeight(tsdf.limits().GetCellIndex({x0, y0}));
    float w10 = tsdf.GetWeight(tsdf.limits().GetCellIndex({x1, y0}));
    float w01 = tsdf.GetWeight(tsdf.limits().GetCellIndex({x0, y1}));
    float w11 = tsdf.GetWeight(tsdf.limits().GetCellIndex({x1, y1}));
    float w_min = std::min(std::min(w00, w10), std::min(w01, w11));
    if (w_min == 0.f) {
      normals.push_back(std::make_pair<float, float>(0.f, 0.f));
      continue;
    }

    float dMdx =
        ((center[1] - y0) * (m11 - m01) + (y1 - center[1]) * (m10 - m00)) /
        (y1 - y0);
    float dMdy =
        ((center[0] - x0) * (m11 - m10) + (x1 - center[0]) * (m01 - m00)) /
        (x1 - x0);

    Eigen::Vector3f sample_normal = {dMdx, dMdy, 0.f};

    const Eigen::Vector3f& estimation_point_to_observation =
        range_data.origin - hit;
    if (sample_normal.dot(estimation_point_to_observation) < 0) {
      //sample_normal = -sample_normal;
      w_min = 0.f;
    }

    normals.push_back(std::make_pair<float, float>(
        Normal2DTo2DAngle({dMdx, dMdy}), static_cast<float>(w_min)));
  }
  return normals;
}

}  // namespace mapping
}  // namespace cartographer
