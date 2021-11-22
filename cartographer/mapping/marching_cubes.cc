//
// Created by bastian on 12.11.21.
//

#include "marching_cubes.h"
#include "math.h"
#include "3d/hybrid_grid_tsdf.h"
#include "3d/submap_3d.h"

namespace cartographer {
namespace mapping {

#ifdef WITH_PCL
pcl::PointXYZ MarchingCubes::InterpolateVertex(float isolevel,
                                               pcl::PointXYZ p1,
                                               pcl::PointXYZ p2,
                                               float valp1,
                                               float valp2) {
  float mu;
  pcl::PointXYZ p;

  // If jump is too big, return a point with x value NAN to indicate it's false
  if (std::abs(valp1 - valp2) > 0.95) {
    p.x = NAN;
    return p;
  }

  if (std::abs(isolevel - valp1) < 1e-5) {
    return p1;
  }
  if (std::abs(isolevel - valp2) < 1e-5) {
    return p2;
  }
  if (std::abs(valp1 - valp2) < 1e-5) {
    p.getArray3fMap() = 0.5 * (p1.getArray3fMap() + p2.getArray3fMap());
    return p;
  }
  mu = (isolevel - valp1) / (valp2 - valp1);
  p.x = p1.x + mu * (p2.x - p1.x);
  p.y = p1.y + mu * (p2.y - p1.y);
  p.z = p1.z + mu * (p2.z - p1.z);
  return p;
}

int MarchingCubes::ProcessCube(Cube &cube,
                               pcl::PointCloud<pcl::PointXYZ> &cloud,
                               float isolevel) {
  int cubeindex = 0;
  if (cube.tsd_values[0] <= isolevel) cubeindex |= 1;
  if (cube.tsd_values[1] <= isolevel) cubeindex |= 2;
  if (cube.tsd_values[2] <= isolevel) cubeindex |= 4;
  if (cube.tsd_values[3] <= isolevel) cubeindex |= 8;
  if (cube.tsd_values[4] <= isolevel) cubeindex |= 16;
  if (cube.tsd_values[5] <= isolevel) cubeindex |= 32;
  if (cube.tsd_values[6] <= isolevel) cubeindex |= 64;
  if (cube.tsd_values[7] <= isolevel) cubeindex |= 128;

  // Cube is entirely in/out of the surface
  if (edge_table_[cubeindex] == 0) {
    return 0;
  }
  pcl::PointXYZ vertices_list[12];

  // Find the points where the surface intersects the cube
  if (edge_table_[cubeindex] & 1) {
    vertices_list[0] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[0],
                          cube.vertice_pos_global[1],
                          cube.tsd_values[0],
                          cube.tsd_values[1]);
  }
  if (edge_table_[cubeindex] & 2) {
    vertices_list[1] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[1],
                          cube.vertice_pos_global[2],
                          cube.tsd_values[1],
                          cube.tsd_values[2]);
  }
  if (edge_table_[cubeindex] & 4) {
    vertices_list[2] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[2],
                          cube.vertice_pos_global[3],
                          cube.tsd_values[2],
                          cube.tsd_values[3]);
  }
  if (edge_table_[cubeindex] & 8) {
    vertices_list[3] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[3],
                          cube.vertice_pos_global[0],
                          cube.tsd_values[3],
                          cube.tsd_values[0]);
  }
  if (edge_table_[cubeindex] & 16) {
    vertices_list[4] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[4],
                          cube.vertice_pos_global[5],
                          cube.tsd_values[4],
                          cube.tsd_values[5]);
  }
  if (edge_table_[cubeindex] & 32) {
    vertices_list[5] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[5],
                          cube.vertice_pos_global[6],
                          cube.tsd_values[5],
                          cube.tsd_values[6]);
  }
  if (edge_table_[cubeindex] & 64) {
    vertices_list[6] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[6],
                          cube.vertice_pos_global[7],
                          cube.tsd_values[6],
                          cube.tsd_values[7]);
  }
  if (edge_table_[cubeindex] & 128) {
    vertices_list[7] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[7],
                          cube.vertice_pos_global[4],
                          cube.tsd_values[7],
                          cube.tsd_values[4]);
  }
  if (edge_table_[cubeindex] & 256) {
    vertices_list[8] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[0],
                          cube.vertice_pos_global[4],
                          cube.tsd_values[0],
                          cube.tsd_values[4]);
  }
  if (edge_table_[cubeindex] & 512) {
    vertices_list[9] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[1],
                          cube.vertice_pos_global[5],
                          cube.tsd_values[1],
                          cube.tsd_values[5]);
  }
  if (edge_table_[cubeindex] & 1024) {
    vertices_list[10] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[2],
                          cube.vertice_pos_global[6],
                          cube.tsd_values[2],
                          cube.tsd_values[6]);
  }
  if (edge_table_[cubeindex] & 2048) {
    vertices_list[11] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[3],
                          cube.vertice_pos_global[7],
                          cube.tsd_values[3],
                          cube.tsd_values[7]);
  }

  // Create the triangle
  int triangle_count = 0;
  pcl::PointXYZ triangle[3];
  for (int i = 0; triangle_table_[cubeindex][i] != -1; i += 3) {
    triangle[0] = vertices_list[triangle_table_[cubeindex][i]];
    triangle[1] = vertices_list[triangle_table_[cubeindex][i + 1]];
    triangle[2] = vertices_list[triangle_table_[cubeindex][i + 2]];
    if (isnan(triangle[0].x) || isnan(triangle[1].x) || isnan(triangle[2].x)) continue;
    cloud.push_back(triangle[0]);
    cloud.push_back(triangle[1]);
    cloud.push_back(triangle[2]);
    triangle_count++;
  }
  return (triangle_count);
}

void MarchingCubes::ProcessTSDFMesh(pcl::PolygonMesh &mesh,
                                    const MapById<SubmapId,
                                                  PoseGraphInterface::SubmapData> &all_submap_data,
                                    const Eigen::Vector3f &robot_position,
                                    bool high_res_mesh,
                                    float cut_off_distance,
                                    float cut_off_height,
                                    float min_weight,
                                    bool all_submaps) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  float isolevel = 0.0f;
  size_t count = 0;
  bool skipped_current_submap = false;

  if (!all_submap_data.empty()) {
    for (auto const &submap_data: all_submap_data) {
      if (!all_submaps && all_submap_data.size() > 1 && !skipped_current_submap) {
        skipped_current_submap = true;
        continue;
      }

      auto submap3d =
          dynamic_cast<const ::cartographer::mapping::Submap3D *>(
              submap_data.data.submap.get());

      const HybridGridTSDF *tsdf;
      if (high_res_mesh) {
        tsdf =
            dynamic_cast<const ::cartographer::mapping::HybridGridTSDF *>(
                &submap3d->high_resolution_hybrid_grid());
      } else {
        tsdf =
            dynamic_cast<const ::cartographer::mapping::HybridGridTSDF *>(
                &submap3d->low_resolution_hybrid_grid());
      }

      float resolution = tsdf->resolution();

      for (auto it = ::cartographer::mapping::HybridGridTSDF::Iterator(*tsdf);
           !it.Done(); it.Next()) {
        const ::cartographer::mapping::TSDFVoxel voxel = it.GetValue();
        const float tsd = tsdf->ValueConverter().ValueToTSD(voxel.discrete_tsd);
        const Eigen::Vector3f cell_center_submap = tsdf->GetCenterOfCell(it.GetCellIndex());
        const Eigen::Vector3f
            cell_center_global = submap3d->local_pose().cast<float>() * cell_center_submap;

        if (voxel.discrete_weight <= min_weight) {
          // Skip inner-object voxels
          continue;
        }

        if (cut_off_distance >= 0.0f &&
            (robot_position.cast<float>() - cell_center_global).norm() > cut_off_distance) {
          // Cut-off cells that are too far away from the robot, if parameter valid (>0)
          continue;
        }

        if (cut_off_height >= 0.0f &&
            cell_center_global.z() - static_cast<float>(robot_position.z()) > cut_off_height) {
          // Cut-off cells that are too high above the robot, if parameter valid (>0)
          continue;
        }

        Cube cube;
        for (int i = 0; i < 8; ++i) {
          cube.vertice_ids[i] = it.GetCellIndex();
          cube.vertice_ids[i].x() += cube.position_arr[i][0];
          cube.vertice_ids[i].y() += cube.position_arr[i][1];
          cube.vertice_ids[i].z() += cube.position_arr[i][2];
          cube.vertice_pos_global[i].x =
              cell_center_global.x() + static_cast<float>(cube.position_arr[i][0]) * resolution;
          cube.vertice_pos_global[i].y =
              cell_center_global.y() + static_cast<float>(cube.position_arr[i][1]) * resolution;
          cube.vertice_pos_global[i].z =
              cell_center_global.z() + static_cast<float>(cube.position_arr[i][2]) * resolution;
          cube.tsd_weights[i] = tsdf->GetWeight(cube.vertice_ids[i]);

          cube.tsd_values[i] =
              cube.tsd_weights[i] <= 0.0f ? tsd : tsdf->GetTSD(cube.vertice_ids[i]);
        }
        count += ProcessCube(cube, cloud, isolevel);

      }
    }
    LOG(INFO) << "[TSDF Mesh] Triangles in Cloud: " << cloud.size() / 3;

    pcl::toPCLPointCloud2(cloud, mesh.cloud);

    for (size_t i = 0; i < count; i++) {
      pcl::Vertices v;
      v.vertices.push_back(i * 3 + 0);
      v.vertices.push_back(i * 3 + 2);
      v.vertices.push_back(i * 3 + 1);
      mesh.polygons.push_back(v);
    }
  }
}
#endif

} //namespace mapping
} // namespace cartographer