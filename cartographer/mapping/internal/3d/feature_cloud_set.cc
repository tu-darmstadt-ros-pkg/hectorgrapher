#include "cartographer/mapping/internal/3d/feature_cloud_set.h"

namespace cartographer {
namespace mapping {

void FeatureCloudSet::Transform(const transform::Rigid3f& transformation) {
  for (auto& cloud : multi_scan_cloud_) {
    cloud = sensor::TransformTimedPointCloud(cloud, transformation);
  }
  plane_feature_locations_ = sensor::TransformTimedPointCloud(
      plane_feature_locations_, transformation);
  edge_feature_locations_ =
      sensor::TransformTimedPointCloud(edge_feature_locations_, transformation);
  //
  //      Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
  //  trans_to_origin.block<3, 1>(0, 3) =
  //  transformation.translation().cast<double>(); trans_to_origin.block<3,
  //  3>(0, 0) = transformation.rotation().toRotationMatrix().cast<double>();
  //    kdtree_edge_->Transform(trans_to_origin);
}

void FeatureCloudSet::ComputeFeatureSet(
    const sensor::TimedPointCloudData& range_data_in_tracking) {
  sensor::TimedPointCloudData range_data_in_sensor =
      sensor::TimedPointCloudData{
          range_data_in_tracking.time, transform::Rigid3f::Identity(),
          sensor::TransformTimedPointCloud(
              range_data_in_tracking.ranges,
              range_data_in_tracking.origin_transform.inverse())};

  float scan_range_rad = common::DegToRad(30);
  std::vector<sensor::TimedPointCloud> filterd_multi_scan_cloud(num_scans_);

  auto cloud = std::make_shared<open3d::geometry::PointCloud>();

  float min_feature_distance = 0.1;
  size_t num_edge_features_per_segment = 6;
  size_t num_plane_features_per_segment = 6;

  for (const auto& point : range_data_in_sensor.ranges) {
    float angle = std::atan2(
        point.position.z(), std::sqrt(point.position.x() * point.position.x() +
                                      point.position.y() * point.position.y()));
    int scan_idx = std::round((angle + scan_range_rad / 2.f) / scan_range_rad *
                              float(num_scans_ - 1));
    if (scan_idx < 0 || scan_idx > 15) {
      LOG(WARNING) << "Degraded point. Angle[deg] " << common::DegToRad(angle)
                   << " \t predicted idx: " << scan_idx;
    } else
      multi_scan_cloud_[scan_idx].push_back({point.position});
  }

  int i = 0;
  for (const auto& cloud : multi_scan_cloud_) {
    ++i;
  }

  // Compute curvature
  for (size_t scan_cloud_idx = 0; scan_cloud_idx < multi_scan_cloud_.size();
       scan_cloud_idx++) {
    const auto& scan_cloud = multi_scan_cloud_[scan_cloud_idx];
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
    std::vector<sensor::TimedPointCloud> segmented_scan_cloud(num_segments_);
    for (const auto& point : scan_cloud) {
      float angle = std::atan2(point.position.y(), point.position.x());
      int segment_idx =
          std::round((angle + M_PI) / (2.0 * M_PI) * float(num_segments_ - 1));
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
        if (candidate.time > plane_threshold_ ||
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
      plane_feature_locations_.insert(plane_feature_locations_.end(),
                                      segment_features.begin(),
                                      segment_features.end());
      segment_features.clear();
      for (int i = scan_cloud_segment.size() - 1; i >= 0; --i) {
        sensor::TimedRangefinderPoint& candidate = scan_cloud_segment[i];
        if (candidate.time < edge_threshold_ ||
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
      edge_feature_locations_.insert(edge_feature_locations_.end(),
                                     segment_features.begin(),
                                     segment_features.end());
    }
  }

  //  std::vector<std::shared_ptr<const open3d::geometry::Geometry>>
  //      render_geometries;
  //  render_geometries.push_back(
  //      open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5));
  //  render_geometries.push_back(cloud);
  //
  //  for (const auto& edge_feature : edge_feature_locations_) {
  //    std::shared_ptr<open3d::geometry::TriangleMesh> sphere =
  //        open3d::geometry::TriangleMesh::CreateSphere(0.1, 20);
  //    Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
  //    trans_to_origin.block<3, 1>(0, 3) =
  //    edge_feature.position.cast<double>();
  //    sphere->Transform(trans_to_origin);
  //    sphere->PaintUniformColor({1.0, 0.0, 0.0});
  //    render_geometries.push_back(sphere);
  //  }
  //  for (const auto& plane_feature : plane_feature_locations_) {
  //    std::shared_ptr<open3d::geometry::TriangleMesh> sphere =
  //        open3d::geometry::TriangleMesh::CreateSphere(0.1, 20);
  //    Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
  //    trans_to_origin.block<3, 1>(0, 3) =
  //    plane_feature.position.cast<double>();
  //    sphere->Transform(trans_to_origin);
  //    sphere->PaintUniformColor({0.0, 1.0, 0.0});
  //    render_geometries.push_back(sphere);
  //  }
  //
  //  open3d::visualization::DrawGeometries(render_geometries);

  Transform(range_data_in_tracking.origin_transform);
}

void FeatureCloudSet::ExtractCorrespondences(
    const sensor::TimedPointCloud& plane_feature_locations,
    const sensor::TimedPointCloud& edge_feature_locations,
    std::vector<std::vector<int>>* edge_correspondences,
    std::vector<std::vector<int>>* plane_correspondences) {
  auto cloud_edge = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : edge_feature_locations_) {
    cloud_edge->points_.emplace_back(point.position.cast<double>());
  }
  auto cloud_plane = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : plane_feature_locations_) {
    cloud_plane->points_.emplace_back(point.position.cast<double>());
  }
  kdtree_edge_ = std::make_shared<open3d::geometry::KDTreeFlann>();
  kdtree_plane_ = std::make_shared<open3d::geometry::KDTreeFlann>();
  kdtree_edge_->SetGeometry(*cloud_edge);
  kdtree_plane_->SetGeometry(*cloud_plane);

  auto cloud_edge_rhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : edge_feature_locations) {
    cloud_edge_rhs->points_.emplace_back(point.position.cast<double>());
  }
  auto cloud_plane_rhs = std::make_shared<open3d::geometry::PointCloud>();
  for (const auto& point : plane_feature_locations) {
    cloud_plane_rhs->points_.emplace_back(point.position.cast<double>());
  }

  //
  cloud_edge->PaintUniformColor({1.0, 0.0, 0.0});
  cloud_edge_rhs->PaintUniformColor({0.5, 0.0, 0.0});
  cloud_plane->PaintUniformColor({0.0, 0.5, 0.0});
  cloud_plane_rhs->PaintUniformColor({0.0, 1.0, 0.0});

  std::vector<std::pair<int, int>> edge_render_correspondences;
  for (int edge_idx = 0; edge_idx < cloud_edge_rhs->points_.size();
       ++edge_idx) {
    std::vector<int> indices_vec(2);
    std::vector<double> dists_vec(2);
    int k = kdtree_edge_->SearchHybrid(cloud_edge_rhs->points_[edge_idx], 0.4,
                                       2, indices_vec, dists_vec);
    if (k == 2) {
      edge_correspondences->push_back(
          {edge_idx, indices_vec[0], indices_vec[1]});
      edge_render_correspondences.emplace_back(edge_idx, indices_vec[0]);
      edge_render_correspondences.emplace_back(edge_idx, indices_vec[1]);
    }
  }
  std::vector<std::pair<int, int>> plane_render_correspondences;
  for (int plane_idx = 0; plane_idx < cloud_plane_rhs->points_.size();
       ++plane_idx) {
    std::vector<int> indices_vec(3);
    std::vector<double> dists_vec(3);
    int k = kdtree_plane_->SearchHybrid(cloud_plane_rhs->points_[plane_idx],
                                        0.5, 3, indices_vec, dists_vec);
    if (k == 3) {
      plane_correspondences->push_back(
          {plane_idx, indices_vec[0], indices_vec[1], indices_vec[2]});
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[0]);
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[1]);
      plane_render_correspondences.emplace_back(plane_idx, indices_vec[2]);
    }
  }
//    auto edge_lineset =
//        open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
//            *cloud_edge_rhs, *cloud_edge, edge_render_correspondences);
//    auto plane_lineset =
//        open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
//            *cloud_plane_rhs, *cloud_plane, plane_render_correspondences);
//
//    open3d::visualization::DrawGeometries({cloud_edge, cloud_edge_rhs,
//                                           cloud_plane, cloud_plane_rhs,
//                                           edge_lineset, plane_lineset});

  //  ceres::Problem problem;
  //  std::array<double, 3> translation = {{0, 0, 0}};
  //  std::array<double, 4> rotation = {{1, 0, 0, 0}};
  //
  //  problem.AddResidualBlock(
  //      new ceres::AutoDiffCostFunction<EdgeFeatureCostFunctor3D,
  //      ceres::DYNAMIC,
  //                                      3, 4>(
  //          new EdgeFeatureCostFunctor3D(1.0, lhs.edge_feature_locations_,
  //                                       rhs.edge_feature_locations_,
  //                                       edge_correspondences),
  //          edge_correspondences.size()),
  //      nullptr, translation.data(), rotation.data());
  //
  //  problem.AddResidualBlock(
  //      new ceres::AutoDiffCostFunction<PlaneFeatureCostFunctor3D,
  //      ceres::DYNAMIC,
  //                                      3, 4>(
  //          new PlaneFeatureCostFunctor3D(1.0, lhs.plane_feature_locations_,
  //                                        rhs.plane_feature_locations_,
  //                                        plane_correspondences),
  //          plane_correspondences.size()),
  //      nullptr, translation.data(), rotation.data());
  //
  //  problem.SetParameterization(rotation.data(),
  //                              new ceres::QuaternionParameterization());
  //
  //  ceres::Solver::Summary summary;
  //  ceres::Solve(ceres_solver_options_, &problem, &summary);
  //  LOG(INFO) << summary.FullReport();
  //
  //  const transform::Rigid3d transform_optimized(
  //      Eigen::Map<const Eigen::Matrix<double, 3, 1>>(translation.data()),
  //      Eigen::Quaterniond(rotation[0], rotation[1], rotation[2],
  //      rotation[3]));
  //
  //  LOG(INFO) << transform_optimized.DebugString();
  //
  //  auto cloud_edge_rhs_optimized =
  //      std::make_shared<open3d::geometry::PointCloud>();
  //  for (const auto& point : rhs.edge_feature_locations_) {
  //    cloud_edge_rhs_optimized->points_.emplace_back(
  //        transform_optimized * point.position.cast<double>());
  //  }
  //  auto cloud_plane_rhs_optimized =
  //      std::make_shared<open3d::geometry::PointCloud>();
  //  for (const auto& point : rhs.plane_feature_locations_) {
  //    cloud_plane_rhs_optimized->points_.emplace_back(
  //        transform_optimized * point.position.cast<double>());
  //  }
  //
  //  cloud_edge_rhs_optimized->PaintUniformColor({0.5, 0.0, 0.0});
  //  cloud_plane_rhs_optimized->PaintUniformColor({0.0, 1.0, 0.0});
  //
  //  auto edge_lineset_optimized =
  //      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
  //          *cloud_edge_rhs_optimized, *cloud_edge_lhs,
  //          edge_render_correspondences);
  //  auto plane_lineset_optimized =
  //      open3d::geometry::LineSet::CreateFromPointCloudCorrespondences(
  //          *cloud_plane_rhs_optimized, *cloud_plane_lhs,
  //          plane_render_correspondences);
  //
  //  open3d::visualization::DrawGeometries(
  //      {cloud_edge_lhs, cloud_edge_rhs_optimized, cloud_plane_lhs,
  //       cloud_plane_rhs_optimized, edge_lineset_optimized,
  //       plane_lineset_optimized});
}

}  // namespace mapping
}  // namespace cartographer
