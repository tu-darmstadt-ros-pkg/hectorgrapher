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

#include "cartographer/mapping/internal/3d/optimizing_local_trajectory_builder.h"

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/time.h"
#include "cartographer/evaluation/grid_drawer.h"
#include "cartographer/mapping/internal/3d/imu_static_calibration.h"
#include "cartographer/mapping/internal/3d/scan_matching/edge_feature_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/plane_feature_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_direct_imu_integration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/prediction_imu_preintegration_cost_functor.h"
#include "cartographer/mapping/internal/3d/scan_matching/relative_translation_and_yaw_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_cost_function.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/tsdf_space_cost_function_3d.h"
#include "cartographer/mapping/proto/3d/optimizing_local_trajectory_builder_options.pb.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "glog/logging.h"

#define USE_OPEN3D
#ifdef USE_OPEN3D
#include "Open3D/Open3D.h"
#endif

namespace cartographer {
namespace mapping {

namespace {
class GravityDirectionCostFunction {
 public:
  GravityDirectionCostFunction(
      const double scaling_factor,
      const Eigen::Quaterniond& gravity_direction_in_tracking)
      : scaling_factor_(scaling_factor),
        gravity_direction_in_tracking_(gravity_direction_in_tracking) {}

  GravityDirectionCostFunction(const GravityDirectionCostFunction&) = delete;
  GravityDirectionCostFunction& operator=(const GravityDirectionCostFunction&) =
      delete;

  template <typename T>
  bool operator()(const T* const orientation, T* residual) const {
    Eigen::Quaternion<T> qorientation(orientation[0], orientation[1],
                                      orientation[2], orientation[3]);

    Eigen::Quaternion<T> delta =
        gravity_direction_in_tracking_.cast<T>() * qorientation;
    residual[0] = scaling_factor_ * transform::GetRoll(delta);
    residual[1] = scaling_factor_ * transform::GetPitch(delta);
    return true;
  }

 private:
  const double scaling_factor_;
  const Eigen::Quaterniond gravity_direction_in_tracking_;
};


}  // namespace

OptimizingLocalTrajectoryBuilder::OptimizingLocalTrajectoryBuilder(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      ceres_solver_options_(common::CreateCeresSolverOptions(
          options.ceres_scan_matcher_options().ceres_solver_options())),
      active_submaps_(options.submaps_options()),
      num_accumulated_(0),
      total_num_accumulated_(0),
      last_optimization_time_(common::FromUniversal(0)),
      initial_data_time_(common::FromUniversal(0)),
      optimization_rate_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .optimization_rate())),
      ct_window_horizon_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_horizon())),
      ct_window_rate_(common::FromSeconds(
          options.optimizing_local_trajectory_builder_options()
              .ct_window_rate())),
      imu_calibrated_(false),
      linear_acceleration_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      angular_velocity_calibration_(
          Eigen::Transform<double, 3, Eigen::Affine>::Identity()),
      motion_filter_(options.motion_filter_options()),
      map_update_enabled_(true) {
  imu_integrator_ = absl::make_unique<ImuIntegrator>(
      options.optimizing_local_trajectory_builder_options().imu_integrator());
}

OptimizingLocalTrajectoryBuilder::~OptimizingLocalTrajectoryBuilder() {}

void OptimizingLocalTrajectoryBuilder::AddImuData(
    const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    imu_data_.push_back(imu_data);
    return;
  }
  initial_data_time_ = imu_data.time;
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
  imu_data_.push_back(imu_data);
}

void OptimizingLocalTrajectoryBuilder::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return;
  }
  if (!imu_data_.empty() && (imu_data_.front().time >= odometry_data.time)) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Odometry data dropped to maintain IMU consistency.";
    return;
  }
  odometer_data_.push_back(odometry_data);
}

std::shared_ptr<FeatureCloudSet>
OptimizingLocalTrajectoryBuilder::ComputeFeatureSet(
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  sensor::TimedPointCloudData range_data_in_sensor =
      sensor::TimedPointCloudData{
          range_data_in_tracking.time, transform::Rigid3f::Identity(),
          sensor::TransformTimedPointCloud(
              range_data_in_tracking.ranges,
              range_data_in_tracking.origin_transform.inverse())};

  size_t num_scans = 16;
  float scan_range_rad = common::DegToRad(30);
  std::vector<sensor::TimedPointCloud> filterd_multi_scan_cloud(num_scans);

  auto cloud = std::make_shared<open3d::geometry::PointCloud>();

  float plane_threshold = 1e-5;
  float edge_threshold = 0.1;
  float min_feature_distance = 0.1;
  size_t num_segments = 4;
  size_t num_edge_features_per_segment = 6;
  size_t num_plane_features_per_segment = 6;

  auto feature_cloud_set = std::make_shared<FeatureCloudSet>(
      plane_threshold, edge_threshold, num_scans, num_segments);

  for (const auto& point : range_data_in_sensor.ranges) {
    float angle = std::atan2(
        point.position.z(), std::sqrt(point.position.x() * point.position.x() +
                                      point.position.y() * point.position.y()));
    int scan_idx = std::round((angle + scan_range_rad / 2.f) / scan_range_rad *
                              float(num_scans - 1));
    CHECK_GE(scan_idx, 0);
    CHECK_GT(num_scans, scan_idx);
    feature_cloud_set->multi_scan_cloud_[scan_idx].push_back({point.position});
  }

  int i = 0;
  for (const auto& cloud : feature_cloud_set->multi_scan_cloud_) {
    ++i;
  }

  // Compute curvature
  for (size_t scan_cloud_idx = 0;
       scan_cloud_idx < feature_cloud_set->multi_scan_cloud_.size();
       scan_cloud_idx++) {
    const auto& scan_cloud =
        feature_cloud_set->multi_scan_cloud_[scan_cloud_idx];
    for (size_t point_idx = 0; point_idx < scan_cloud.size(); point_idx++) {
      if (point_idx < 6 || point_idx >= scan_cloud.size() - 6) continue;
      auto point = scan_cloud[point_idx];
      int i = point_idx;
      float diff_x =
          scan_cloud[i - 5].position.x() + scan_cloud[i - 4].position.x() +
          scan_cloud[i - 3].position.x() + scan_cloud[i - 2].position.x() +
          scan_cloud[i - 1].position.x() + scan_cloud[i + 5].position.x() +
          scan_cloud[i + 4].position.x() + scan_cloud[i + 3].position.x() +
          scan_cloud[i + 2].position.x() + scan_cloud[i + 1].position.x() -
          10 * scan_cloud[i].position.x();
      float diff_y =
          scan_cloud[i - 5].position.y() + scan_cloud[i - 4].position.y() +
          scan_cloud[i - 3].position.y() + scan_cloud[i - 2].position.y() +
          scan_cloud[i - 1].position.y() + scan_cloud[i + 5].position.y() +
          scan_cloud[i + 4].position.y() + scan_cloud[i + 3].position.y() +
          scan_cloud[i + 2].position.y() + scan_cloud[i + 1].position.y() -
          10 * scan_cloud[i].position.y();
      float diff_z =
          scan_cloud[i - 5].position.z() + scan_cloud[i - 4].position.z() +
          scan_cloud[i - 3].position.z() + scan_cloud[i - 2].position.z() +
          scan_cloud[i - 1].position.z() + scan_cloud[i + 5].position.z() +
          scan_cloud[i + 4].position.z() + scan_cloud[i + 3].position.z() +
          scan_cloud[i + 2].position.z() + scan_cloud[i + 1].position.z() -
          10 * scan_cloud[i].position.z();
      float point_curvature =
          diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
      point_curvature /= scan_cloud[i].position.norm();

      float angle = std::atan2(scan_cloud[i + 5].position.y(),
                               scan_cloud[i + 5].position.x()) -
                    std::atan2(scan_cloud[i - 5].position.y(),
                               scan_cloud[i - 5].position.x());
      bool angle_continuous = (std::abs(common::RadToDeg(angle)) < 2.1);
      bool depth_continuous = true;

      Eigen::Vector3f scan_direction = scan_cloud[i].position.normalized();
      for (int i_depth = -5; i_depth <= 5; ++i_depth) {
        if (std::abs((scan_cloud[i - i_depth].position -
                      scan_cloud[i - i_depth + 1].position)
                         .dot(scan_direction)) > 0.2)
          depth_continuous = false;
      }

      if (angle_continuous && depth_continuous) {
        filterd_multi_scan_cloud[scan_cloud_idx].push_back(
            {scan_cloud[i].position, point_curvature});
        double r = point_curvature / 0.05;
        r = common::Clamp(r, 0.0, 1.0);
        cloud->colors_.emplace_back(r, 1.0 - r, 0.1);
        cloud->points_.emplace_back(point.position[0], point.position[1],
                                    point.position[2]);
      }
    }
  }

  for (const auto& scan_cloud : filterd_multi_scan_cloud) {
    std::vector<sensor::TimedPointCloud> segmented_scan_cloud(num_segments);
    for (const auto& point : scan_cloud) {
      float angle = std::atan2(point.position.y(), point.position.x());
      int segment_idx =
          std::round((angle + M_PI) / (2.0 * M_PI) * float(num_segments - 1));
      segmented_scan_cloud[segment_idx].push_back(point);
    }
    for (auto& scan_cloud_segment : segmented_scan_cloud) {
      if (scan_cloud_segment.empty()) continue;
      std::sort(scan_cloud_segment.begin(), scan_cloud_segment.end(),
                [](sensor::TimedRangefinderPoint lhs,
                   sensor::TimedRangefinderPoint rhs) {
                  return lhs.time < rhs.time;
                });

      sensor::TimedPointCloud segment_features;
      // plane features
      for (const auto& candidate : scan_cloud_segment) {
        if (candidate.time > plane_threshold ||
            segment_features.size() == num_plane_features_per_segment)
          break;
        bool distance_check = true;
        for (const auto& point : segment_features) {
          if ((candidate.position - point.position).norm() <
              min_feature_distance)
            distance_check = false;
        }
        if (distance_check) segment_features.push_back(candidate);
      }
      feature_cloud_set->plane_feature_locations_.insert(
          feature_cloud_set->plane_feature_locations_.end(),
          segment_features.begin(), segment_features.end());
      segment_features.clear();
      for (int i = scan_cloud_segment.size() - 1; i >= 0; --i) {
        sensor::TimedRangefinderPoint& candidate = scan_cloud_segment[i];
        if (candidate.time < edge_threshold ||
            segment_features.size() == num_edge_features_per_segment)
          break;
        bool distance_check = true;
        for (const auto& point : segment_features) {
          if ((candidate.position - point.position).norm() <
              min_feature_distance)
            distance_check = false;
        }
        if (distance_check) segment_features.push_back(candidate);
      }
      feature_cloud_set->edge_feature_locations_.insert(
          feature_cloud_set->edge_feature_locations_.end(),
          segment_features.begin(), segment_features.end());
    }
  }

  std::vector<std::shared_ptr<const open3d::geometry::Geometry>>
      render_geometries;
  render_geometries.push_back(
      open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5));
  render_geometries.push_back(cloud);

  for (const auto& edge_feature : feature_cloud_set->edge_feature_locations_) {
    std::shared_ptr<open3d::geometry::TriangleMesh> sphere =
        open3d::geometry::TriangleMesh::CreateSphere(0.1, 20);
    Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
    trans_to_origin.block<3, 1>(0, 3) = edge_feature.position.cast<double>();
    sphere->Transform(trans_to_origin);
    sphere->PaintUniformColor({1.0, 0.0, 0.0});
    render_geometries.push_back(sphere);
  }
  for (const auto& plane_feature :
       feature_cloud_set->plane_feature_locations_) {
    std::shared_ptr<open3d::geometry::TriangleMesh> sphere =
        open3d::geometry::TriangleMesh::CreateSphere(0.1, 20);
    Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
    trans_to_origin.block<3, 1>(0, 3) = plane_feature.position.cast<double>();
    sphere->Transform(trans_to_origin);
    sphere->PaintUniformColor({0.0, 1.0, 0.0});
    render_geometries.push_back(sphere);
  }

  open3d::visualization::DrawGeometries(render_geometries);

  feature_cloud_set->Transform(range_data_in_tracking.origin_transform);

  return feature_cloud_set;
}

void OptimizingLocalTrajectoryBuilder::MatchFeatureSets(
    const FeatureCloudSet& lhs, const FeatureCloudSet& rhs) {
  auto cloud_edge_lhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : lhs.edge_feature_locations_) {
    cloud_edge_lhs->points_.emplace_back(point.position.cast<double>());
  }
  auto cloud_plane_lhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : lhs.plane_feature_locations_) {
    cloud_plane_lhs->points_.emplace_back(point.position.cast<double>());
  }
  auto cloud_edge_rhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : rhs.edge_feature_locations_) {
    cloud_edge_rhs->points_.emplace_back(point.position.cast<double>());
  }
  auto cloud_plane_rhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : rhs.plane_feature_locations_) {
    cloud_plane_rhs->points_.emplace_back(point.position.cast<double>());
  }

  open3d::geometry::KDTreeFlann kdtree_edge_lhs;
  open3d::geometry::KDTreeFlann kdtree_plane_lhs;
  kdtree_edge_lhs.SetGeometry(*cloud_edge_lhs);
  kdtree_plane_lhs.SetGeometry(*cloud_plane_lhs);

  cloud_edge_lhs->PaintUniformColor({1.0, 0.0, 0.0});
  cloud_edge_rhs->PaintUniformColor({0.5, 0.0, 0.0});
  cloud_plane_lhs->PaintUniformColor({0.0, 0.5, 0.0});
  cloud_plane_rhs->PaintUniformColor({0.0, 1.0, 0.0});

  std::vector<std::vector<int>> edge_correspondences;
  std::vector<std::pair<int, int>> edge_render_correspondences;
  for (int edge_idx = 0; edge_idx < cloud_edge_rhs->points_.size();
       ++edge_idx) {
    std::vector<int> indices_vec(2);
    std::vector<double> dists_vec(2);
    int k = kdtree_edge_lhs.SearchHybrid(cloud_edge_rhs->points_[edge_idx], 0.4,
                                         2, indices_vec, dists_vec);
    if (k == 2) {
      edge_correspondences.push_back(
          {edge_idx, indices_vec[0], indices_vec[1]});
      edge_render_correspondences.emplace_back(edge_idx, indices_vec[0]);
      edge_render_correspondences.emplace_back(edge_idx, indices_vec[1]);
    }
  }
  std::vector<std::vector<int>> plane_correspondences;
  std::vector<std::pair<int, int>> plane_render_correspondences;
  for (int plane_idx = 0; plane_idx < cloud_plane_rhs->points_.size();
       ++plane_idx) {
    std::vector<int> indices_vec(3);
    std::vector<double> dists_vec(3);
    int k = kdtree_plane_lhs.SearchHybrid(cloud_plane_rhs->points_[plane_idx],
                                          0.5, 3, indices_vec, dists_vec);
    if (k == 3) {
      plane_correspondences.push_back(
          {plane_idx, indices_vec[0], indices_vec[1], indices_vec[2]});
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[0]);
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[1]);
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[2]);
    }
  }
  auto edge_lineset =
      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
          *cloud_edge_rhs, *cloud_edge_lhs, edge_render_correspondences);
  auto plane_lineset =
      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
          *cloud_plane_rhs, *cloud_plane_lhs, plane_render_correspondences);

  open3d::visualization::DrawGeometries({cloud_edge_lhs, cloud_edge_rhs,
                                         cloud_plane_lhs, cloud_plane_rhs,
                                         edge_lineset, plane_lineset});

  ceres::Problem problem;
  std::array<double, 3> translation = {{0, 0, 0}};
  std::array<double, 4> rotation = {{1, 0, 0, 0}};

  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<EdgeFeatureCostFunctor3D, ceres::DYNAMIC,
                                      3, 4>(
          new EdgeFeatureCostFunctor3D(1.0, lhs.edge_feature_locations_,
                                       rhs.edge_feature_locations_,
                                       edge_correspondences),
          edge_correspondences.size()),
      nullptr, translation.data(), rotation.data());

  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PlaneFeatureCostFunctor3D, ceres::DYNAMIC,
                                      3, 4>(
          new PlaneFeatureCostFunctor3D(1.0, lhs.plane_feature_locations_,
                                        rhs.plane_feature_locations_,
                                        plane_correspondences),
          plane_correspondences.size()),
      nullptr, translation.data(), rotation.data());

  problem.SetParameterization(rotation.data(),
                              new ceres::QuaternionParameterization());

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_solver_options_, &problem, &summary);
  LOG(INFO) << summary.FullReport();

  const transform::Rigid3d transform_optimized(
      Eigen::Map<const Eigen::Matrix<double, 3, 1>>(translation.data()),
      Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]));

  LOG(INFO) << transform_optimized.DebugString();

  auto cloud_edge_rhs_optimized =
      std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : rhs.edge_feature_locations_) {
    cloud_edge_rhs_optimized->points_.emplace_back(
        transform_optimized * point.position.cast<double>());
  }
  auto cloud_plane_rhs_optimized =
      std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : rhs.plane_feature_locations_) {
    cloud_plane_rhs_optimized->points_.emplace_back(
        transform_optimized * point.position.cast<double>());
  }

  cloud_edge_rhs_optimized->PaintUniformColor({0.5, 0.0, 0.0});
  cloud_plane_rhs_optimized->PaintUniformColor({0.0, 1.0, 0.0});

  auto edge_lineset_optimized =
      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
          *cloud_edge_rhs_optimized, *cloud_edge_lhs,
          edge_render_correspondences);
  auto plane_lineset_optimized =
      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
          *cloud_plane_rhs_optimized, *cloud_plane_lhs,
          plane_render_correspondences);

  open3d::visualization::DrawGeometries(
      {cloud_edge_lhs, cloud_edge_rhs_optimized, cloud_plane_lhs,
       cloud_plane_rhs_optimized, edge_lineset_optimized,
       plane_lineset_optimized});
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  CHECK_GT(range_data_in_tracking.ranges.size(), 0);

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }

  feature_sets_in_tracking_.push_back(
      ComputeFeatureSet(range_data_in_tracking));

  if (feature_sets_in_tracking_.size() > 1) {
    MatchFeatureSets(*feature_sets_in_tracking_[0],
                     *feature_sets_in_tracking_[1]);
  }

  PointCloudSet point_cloud_set;
  point_cloud_set.time = range_data_in_tracking.time;
  point_cloud_set.origin = range_data_in_tracking.origin();
  for (const auto& hit : range_data_in_tracking.ranges) {
    const Eigen::Vector3f delta =
        hit.position - range_data_in_tracking.origin();
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        point_cloud_set.points.push_back({hit.position});
      }
    }
  }

  auto high_resolution_options =
      options_.high_resolution_adaptive_voxel_filter_options();
  high_resolution_options.set_min_num_points(
      high_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter high_resolution_adaptive_voxel_filter(
      high_resolution_options);
  point_cloud_set.high_resolution_filtered_points =
      high_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);

  auto low_resolution_options =
      options_.low_resolution_adaptive_voxel_filter_options();
  low_resolution_options.set_min_num_points(
      low_resolution_options.min_num_points() /
      options_.num_accumulated_range_data());
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      low_resolution_options);
  point_cloud_set.low_resolution_filtered_points =
      low_resolution_adaptive_voxel_filter.Filter(point_cloud_set.points);
  point_cloud_data_.push_back(point_cloud_set);

  ++num_accumulated_;
  ++total_num_accumulated_;

  return MaybeOptimize(range_data_in_tracking.time);
}

void OptimizingLocalTrajectoryBuilder::AddControlPoint(common::Time t) {
  if (control_points_.empty()) {
    if (options_.optimizing_local_trajectory_builder_options()
        .initialize_map_orientation_with_imu()) {
      Eigen::Quaterniond g = extrapolator_->EstimateGravityOrientation(t);
      LOG(INFO) << "g " << g.vec();
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), g, Eigen::Vector3d::Zero())});
    } else {
      control_points_.push_back(ControlPoint{
          t, State(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                   Eigen::Vector3d::Zero())});
    }
  } else {
    if (active_submaps_.submaps().empty()) {
      control_points_.push_back(
          ControlPoint{t, control_points_.back().state});
    } else {
      control_points_.push_back(
          ControlPoint{t, PredictState(control_points_.back().state,
                                       control_points_.back().time, t)});
    }
  }
}

void OptimizingLocalTrajectoryBuilder::RemoveObsoleteSensorData() {
  if (control_points_.empty()) {
    return;
  }

  while (ct_window_horizon_ <
      control_points_.back().time - control_points_.front().time) {
    control_points_.pop_front();
  }

  while (point_cloud_data_.size() > 0 &&
      (point_cloud_data_.front().time < control_points_.front().time)) {
    point_cloud_data_.pop_front();
  }
  while (imu_data_.size() > 1 &&
      (imu_data_[1].time <= control_points_.front().time)) {
    imu_data_.pop_front();
  }

  while (odometer_data_.size() > 1 &&
         (odometer_data_[1].time <= control_points_.front().time)) {
    odometer_data_.pop_front();
  }
}

void OptimizingLocalTrajectoryBuilder::TransformStates(
    const transform::Rigid3d& transform) {
  for (ControlPoint& control_point : control_points_) {
    const transform::Rigid3d new_pose =
        transform * control_point.state.ToRigid();
    const auto& velocity = control_point.state.velocity;
    const Eigen::Vector3d new_velocity =
        transform.rotation() *
        Eigen::Vector3d(velocity[0], velocity[1], velocity[2]);
    control_point.state =
        State(new_pose.translation(), new_pose.rotation(), new_velocity);
  }
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::MaybeOptimize(const common::Time time) {
  if (time - initial_data_time_ < 2.0 * ct_window_horizon_ ||
      time - last_optimization_time_ < optimization_rate_) {
    if (time - initial_data_time_ < 2.0 * ct_window_horizon_) {
      LOG(INFO) << "No Optimization - not enough time since initialization "
                << common::ToSeconds(time - initial_data_time_) << "\t < "
                << 2.0 * common::ToSeconds(ct_window_horizon_);
    }
    return nullptr;
  }
  if(control_points_.empty()) {
    AddControlPoint(initial_data_time_);
  }
  last_optimization_time_ = time;
  if (!imu_calibrated_ &&
      options_.optimizing_local_trajectory_builder_options()
          .calibrate_imu()) {
    CalibrateIMU(imu_data_, gravity_constant_,
                 linear_acceleration_calibration_,
                 angular_velocity_calibration_);
    imu_integrator_->SetLinearAccelerationCalibration(
        linear_acceleration_calibration_);
    imu_integrator_->SetAngularVelocityCalibration(
        angular_velocity_calibration_);
    imu_calibrated_ = true;

    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    sensor::ImuData debiased_imu_data = imu_data_.front();
    debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
    debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
    extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
        ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
        options_.imu_gravity_time_constant(), debiased_imu_data);
    for(size_t i = 1; i < imu_data_.size(); ++i) {
      debiased_imu_data = imu_data_[i];
      debiased_imu_data.linear_acceleration = linear_acceleration_calibration_ * debiased_imu_data.linear_acceleration;
      debiased_imu_data.angular_velocity = angular_velocity_calibration_ * debiased_imu_data.angular_velocity;
      extrapolator_->AddImuData(debiased_imu_data);
    }

  }
  if(options_.optimizing_local_trajectory_builder_options().sync_control_points_with_range_data()) {
    for (size_t i = 0; i < point_cloud_data_.size(); ++i) {
      PointCloudSet &point_cloud_set = point_cloud_data_[i];
      if(control_points_.back().time < point_cloud_set.time &&point_cloud_set.time < imu_data_.back().time) AddControlPoint(point_cloud_set.time);
    }
  }
  else {
    while (control_points_.back().time + ct_window_rate_ <
        imu_data_.back().time) {
      AddControlPoint(control_points_.back().time + ct_window_rate_);
    }
  }

  ceres::Problem problem;
  if (!active_submaps_.submaps().empty()
  ) {
    std::shared_ptr<const Submap3D> matching_submap =
        active_submaps_.submaps().front();


    // We assume the map is always aligned with the direction of gravity
    CHECK(matching_submap->local_pose().inverse().rotation().isApprox(Eigen::Quaterniond::Identity(), 1e-8));
    Eigen::Vector3d gravity =
        gravity_constant_ * Eigen::Vector3d::UnitZ();
    // We transform the states in 'control_points_' in place to be in the submap
    // frame as expected by the OccupiedSpaceCostFunctor. This is reverted after
    // solving the optimization problem.
    TransformStates(matching_submap->local_pose().inverse());
    auto next_control_point = control_points_.begin();
    for (size_t i = 0; i < point_cloud_data_.size(); ++i) {
      PointCloudSet& point_cloud_set = point_cloud_data_[i];
      if (point_cloud_set.time < control_points_.back().time) {
        while (next_control_point->time <= point_cloud_set.time) {
          CHECK(next_control_point != control_points_.end());
          next_control_point++;
        }
        CHECK(next_control_point != control_points_.begin());
        CHECK_LE(std::prev(next_control_point)->time, point_cloud_set.time);
        CHECK_GE(next_control_point->time, point_cloud_set.time);
        const double duration = common::ToSeconds(
            next_control_point->time - std::prev(next_control_point)->time);
        const double interpolation_factor =
            common::ToSeconds(point_cloud_set.time -
                              std::prev(next_control_point)->time) /
            duration;
        switch (matching_submap->high_resolution_hybrid_grid().GetGridType()) {
          case GridType::PROBABILITY_GRID: {
            problem.AddResidualBlock(
                scan_matching::InterpolatedOccupiedSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                                .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                point_cloud_set.high_resolution_filtered_points
                                    .size())),
                        point_cloud_set.high_resolution_filtered_points,
                        static_cast<const HybridGrid&>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                nullptr,
                std::prev(next_control_point)->state.translation.data(),
                std::prev(next_control_point)->state.rotation.data(),
                next_control_point->state.translation.data(),
                next_control_point->state.rotation.data());
            break;
          }
          case GridType::TSDF: {
            if (options_.optimizing_local_trajectory_builder_options()
                    .high_resolution_grid_weight() > 0.0) {
              if(std::prev(next_control_point)->time == point_cloud_set.time) {
              problem.AddResidualBlock(
                  scan_matching::TSDFSpaceCostFunction3D::
                  CreateAutoDiffCostFunction(
                      options_.optimizing_local_trajectory_builder_options()
                          .high_resolution_grid_weight() /
                          std::sqrt(static_cast<double>(
                                        point_cloud_set
                                            .high_resolution_filtered_points.size())),
                      point_cloud_set.high_resolution_filtered_points,
                      static_cast<const HybridGridTSDF&>(
                          matching_submap->high_resolution_hybrid_grid())),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data());
              }
              else {
                problem.AddResidualBlock(
                    scan_matching::InterpolatedTSDFSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                            .high_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                          point_cloud_set
                                              .high_resolution_filtered_points.size())),
                        point_cloud_set.high_resolution_filtered_points,
                        static_cast<const HybridGridTSDF &>(
                            matching_submap->high_resolution_hybrid_grid()),
                        interpolation_factor),
                    nullptr,
                    std::prev(next_control_point)->state.translation.data(),
                    std::prev(next_control_point)->state.rotation.data(),
                    next_control_point->state.translation.data(),
                    next_control_point->state.rotation.data());
              }
            }
            if (options_.optimizing_local_trajectory_builder_options()
                    .low_resolution_grid_weight() > 0.0) {

              if(std::prev(next_control_point)->time == point_cloud_set.time) {
                problem.AddResidualBlock(
                    scan_matching::TSDFSpaceCostFunction3D::
                    CreateAutoDiffCostFunction(
                        options_.optimizing_local_trajectory_builder_options()
                            .low_resolution_grid_weight() /
                            std::sqrt(static_cast<double>(
                                          point_cloud_set
                                              .low_resolution_filtered_points.size())),
                        point_cloud_set.low_resolution_filtered_points,
                        static_cast<const HybridGridTSDF&>(
                            matching_submap->low_resolution_hybrid_grid())),
                    nullptr,
                    std::prev(next_control_point)->state.translation.data(),
                    std::prev(next_control_point)->state.rotation.data());
              }
              else {
              problem.AddResidualBlock(
                  scan_matching::InterpolatedTSDFSpaceCostFunction3D::
                      CreateAutoDiffCostFunction(
                          options_.optimizing_local_trajectory_builder_options()
                                  .low_resolution_grid_weight() /
                              std::sqrt(static_cast<double>(
                                  point_cloud_set.low_resolution_filtered_points
                                      .size())),
                          point_cloud_set.low_resolution_filtered_points,
                          static_cast<const HybridGridTSDF&>(
                              matching_submap->low_resolution_hybrid_grid()),
                          interpolation_factor),
                  nullptr,
                  std::prev(next_control_point)->state.translation.data(),
                  std::prev(next_control_point)->state.rotation.data(),
                  next_control_point->state.translation.data(),
                  next_control_point->state.rotation.data());
            }
            }
            break;
          }
          case GridType::NONE:
            LOG(FATAL) << "Gridtype not initialized.";
            break;
        }
      }
    }

    switch (options_.optimizing_local_trajectory_builder_options()
                .imu_cost_term()) {
      case proto::DIRECT: {
        for (size_t i = 1; i < control_points_.size(); ++i) {
          problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<
                  PredictionDirectImuIntegrationCostFunctor, 9, 3, 3, 4, 3, 3,
                  4>(new PredictionDirectImuIntegrationCostFunctor(
                  options_.optimizing_local_trajectory_builder_options()
                      .translation_weight(),
                  options_.optimizing_local_trajectory_builder_options()
                      .velocity_weight(),
                  options_.optimizing_local_trajectory_builder_options()
                      .rotation_weight(),
                  common::ToSeconds(control_points_[i].time -
                                    control_points_[i - 1].time),
                  imu_data_, linear_acceleration_calibration_,
                  angular_velocity_calibration_, control_points_[i - 1].time,
                  control_points_[i].time,
                  options_.optimizing_local_trajectory_builder_options()
                      .imu_integrator())),
              nullptr, control_points_[i - 1].state.translation.data(),
              control_points_[i - 1].state.velocity.data(),
              control_points_[i - 1].state.rotation.data(),
              control_points_[i].state.translation.data(),
              control_points_[i].state.velocity.data(),
              control_points_[i].state.rotation.data());
        }
        break;
      }
      case proto::PREINTEGRATION: {
        auto it = --imu_data_.cend();
        while (it->time > control_points_.begin()->time) {
          CHECK(it != imu_data_.cbegin());
          --it;
        }
        for (size_t i = 1; i < control_points_.size(); ++i) {
          IntegrateImuWithTranslationResult<double> result =
              imu_integrator_->IntegrateIMU(imu_data_,
                                            control_points_[i - 1].time,
                                            control_points_[i].time, &it);

          problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<
                  PredictionImuPreintegrationCostFunctor, 9, 3, 3, 4, 3, 3, 4>(
                  new PredictionImuPreintegrationCostFunctor(
                      options_.optimizing_local_trajectory_builder_options()
                          .translation_weight(),
                      options_.optimizing_local_trajectory_builder_options()
                          .velocity_weight(),
                      options_.optimizing_local_trajectory_builder_options()
                          .rotation_weight(),
                      common::ToSeconds(control_points_[i].time -
                                        control_points_[i - 1].time),
                      result,
                      gravity)),
              nullptr, control_points_[i - 1].state.translation.data(),
              control_points_[i - 1].state.velocity.data(),
              control_points_[i - 1].state.rotation.data(),
              control_points_[i].state.translation.data(),
              control_points_[i].state.velocity.data(),
              control_points_[i].state.rotation.data());
        }
        break;
      }
      default:
        LOG(FATAL) << "Unknown proto::IMUCostTerm";
    }

    if (odometer_data_.size() > 1) {
      transform::TransformInterpolationBuffer interpolation_buffer;
      for (const auto& odometer_data : odometer_data_) {
        interpolation_buffer.Push(odometer_data.time, odometer_data.pose);
      }
      for (size_t i = 1; i < control_points_.size(); ++i) {
        // Only add constraints for this range data if  we have bracketing data
        // from the odometer.
        if (!(interpolation_buffer.earliest_time() <=
                  control_points_[i - 1].time &&
              control_points_[i].time <= interpolation_buffer.latest_time())) {
          continue;
        }
        const transform::Rigid3d previous_odometer_pose =
            interpolation_buffer.Lookup(control_points_[i - 1].time);
        const transform::Rigid3d current_odometer_pose =
            interpolation_buffer.Lookup(control_points_[i].time);
        const transform::Rigid3d delta_pose =
            current_odometer_pose.inverse() * previous_odometer_pose;
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                RelativeTranslationAndYawCostFunction, 6, 3, 4, 3, 4>(
                new RelativeTranslationAndYawCostFunction(
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_translation_weight(),
                    options_.optimizing_local_trajectory_builder_options()
                        .odometry_rotation_weight(),
                    delta_pose)),
            nullptr, control_points_[i - 1].state.translation.data(),
            control_points_[i - 1].state.rotation.data(),
            control_points_[i].state.translation.data(),
            control_points_[i].state.rotation.data());
      }
    }

    problem.SetParameterBlockConstant(
        control_points_.front().state.translation.data());
    problem.SetParameterBlockConstant(
        control_points_.front().state.rotation.data());
    problem.SetParameterBlockConstant(
        control_points_.front().state.velocity.data());

    for (size_t i = 1; i < control_points_.size(); ++i) {
      problem.SetParameterization(control_points_[i].state.rotation.data(),
                                  new ceres::QuaternionParameterization());

      problem.SetParameterBlockConstant(
          control_points_[i].state.rotation.data());
    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_solver_options_, &problem, &summary);
    // The optimized states in 'control_points_' are in the submap frame and we
    // transform them in place to be in the local SLAM frame again.
    TransformStates(matching_submap->local_pose());
  }


  num_accumulated_ = 0;
  const transform::Rigid3d optimized_pose =
      control_points_.front().state.ToRigid();
  extrapolator_->AddPose(control_points_.front().time, optimized_pose);
  sensor::RangeData accumulated_range_data_in_tracking = {
      Eigen::Vector3f::Zero(), {}, {}};

  if (active_submaps_.submaps().empty()) {
    //To initialize the empty map we add all available range data assuming zero motion.
    std::deque<ControlPoint>::iterator control_points_iterator =
        control_points_.begin();
    for (auto& point_cloud_set : point_cloud_data_) {
      if (point_cloud_set.time < control_points_.back().time) {
        while (control_points_iterator->time <= point_cloud_set.time) {
          ++control_points_iterator;
        }
        CHECK(control_points_iterator != control_points_.begin());
        CHECK(control_points_iterator != control_points_.end());
        auto transform_cloud = InterpolateTransform(
            std::prev(control_points_iterator)->state.ToRigid(),
            control_points_iterator->state.ToRigid(),
            std::prev(control_points_iterator)->time,
            control_points_iterator->time, point_cloud_set.time);
        const transform::Rigid3f transform =
            (optimized_pose.inverse() * transform_cloud).cast<float>();
        for (const auto& point : point_cloud_set.points) {
          accumulated_range_data_in_tracking.returns.push_back(transform *
                                                               point);
        }
        accumulated_range_data_in_tracking.origin =
            (transform * point_cloud_set.origin);
      }
    }
  } else {
    CHECK(control_points_.front().time <= point_cloud_data_.front().time);
    while (ct_window_horizon_ <
           control_points_.back().time - point_cloud_data_.front().time) {
      while (std::next(control_points_.begin())->time <
             point_cloud_data_.front().time) {
        control_points_.pop_front();
      }
      CHECK(std::next(control_points_.begin()) != control_points_.end());
      auto transform_cloud = InterpolateTransform(
          control_points_.begin()->state.ToRigid(),
          std::next(control_points_.begin())->state.ToRigid(),
          control_points_.begin()->time,
          std::next(control_points_.begin())->time,
          point_cloud_data_.front().time);
      const transform::Rigid3f transform =
          (optimized_pose.inverse() * transform_cloud).cast<float>();
      for (const auto& point : point_cloud_data_.front().points) {
        accumulated_range_data_in_tracking.returns.push_back(transform * point);
      }
      accumulated_range_data_in_tracking.origin =
          (transform * point_cloud_data_.front().origin);
      point_cloud_data_.pop_front();
    }
  }

  RemoveObsoleteSensorData();

  return AddAccumulatedRangeData(time, optimized_pose,
                                 accumulated_range_data_in_tracking);
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::MatchingResult>
OptimizingLocalTrajectoryBuilder::AddAccumulatedRangeData(
    const common::Time time, const transform::Rigid3d& optimized_pose,
    const sensor::RangeData& range_data_in_tracking) {
  if (range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  sensor::RangeData filtered_range_data_in_tracking = {
      range_data_in_tracking.origin,
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.returns),
      sensor::VoxelFilter(options_.voxel_filter_size())
          .Filter(range_data_in_tracking.misses)};

  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, optimized_pose.cast<float>());

  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  const Eigen::Quaterniond gravity_alignment = optimized_pose.rotation();
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, optimized_pose,
      gravity_alignment);

  return absl::make_unique<MatchingResult>(MatchingResult{
      time, optimized_pose, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

std::unique_ptr<OptimizingLocalTrajectoryBuilder::InsertionResult>
OptimizingLocalTrajectoryBuilder::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  const Eigen::VectorXf rotational_scan_matcher_histogram_in_gravity =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());

  const Eigen::Quaterniond local_from_gravity_aligned =
      pose_estimate.rotation() * gravity_alignment.inverse();
  if (!map_update_enabled_) {
    LOG(WARNING) << "Map Update Disabled!";
  }
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps =
      map_update_enabled_
          ? active_submaps_.InsertData(
                filtered_range_data_in_local, local_from_gravity_aligned,
                rotational_scan_matcher_histogram_in_gravity)
          : active_submaps_.submaps();
  return absl::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram_in_gravity,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

State OptimizingLocalTrajectoryBuilder::PredictState(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  switch (
      options_.optimizing_local_trajectory_builder_options().imu_integrator()) {
    case proto::IMUIntegrator::EULER:
      return PredictStateEuler(start_state, start_time, end_time);
    case proto::IMUIntegrator::RK4:
      return PredictStateRK4(start_state, start_time, end_time);
    default:
      LOG(FATAL) << "Unsupported imu integrator type.";
  }
}

State OptimizingLocalTrajectoryBuilder::PredictStateRK4(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }
  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateStateWithGravity(imu_data_, start_state,
                                                 start_time, end_time, &it);
  return State(result.delta_translation, result.delta_rotation,
               result.delta_velocity);
}

State OptimizingLocalTrajectoryBuilder::PredictStateEuler(
    const State& start_state, const common::Time start_time,
    const common::Time end_time) {
  auto it = --imu_data_.cend();
  while (it->time > start_time) {
    CHECK(it != imu_data_.cbegin());
    --it;
  }

  const IntegrateImuWithTranslationResult<double> result =
      imu_integrator_->IntegrateIMU(imu_data_, start_time, end_time, &it);

  const Eigen::Quaterniond start_rotation(
      start_state.rotation[0], start_state.rotation[1], start_state.rotation[2],
      start_state.rotation[3]);
  const Eigen::Quaterniond orientation = start_rotation * result.delta_rotation;
  const double delta_time_seconds = common::ToSeconds(end_time - start_time);

  const Eigen::Vector3d position =
      Eigen::Map<const Eigen::Vector3d>(start_state.translation.data()) +
      delta_time_seconds *
          Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data());
  const Eigen::Vector3d velocity =
      Eigen::Map<const Eigen::Vector3d>(start_state.velocity.data()) +
      start_rotation * result.delta_velocity -
      gravity_constant_ * delta_time_seconds * Eigen::Vector3d::UnitZ();

  return State(position, orientation, velocity);
}


void OptimizingLocalTrajectoryBuilder::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  LOG(WARNING)
      << "OptimizingLocalTrajectoryBuilder::RegisterMetrics not implemented";
}

void OptimizingLocalTrajectoryBuilder::SetMapUpdateEnabled(
    bool map_update_enabled) {
  map_update_enabled_ = map_update_enabled;
}

}  // namespace mapping
}  // namespace cartographer
