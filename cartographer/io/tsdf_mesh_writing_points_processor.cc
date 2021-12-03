//
// Created by bastian on 12.11.21.
//

#include "tsdf_mesh_writing_points_processor.h"
#include <boost/filesystem.hpp>
#include <utility>
#include <pcl/PolygonMesh.h>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "../mapping/3d/submap_3d.h"
#include "cartographer/mapping/marching_cubes.h"

namespace cartographer {
namespace io {
std::unique_ptr<TsdfMeshWritingPointsProcessor>
TsdfMeshWritingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary *const dictionary,
    PointsProcessor *const next) {
  return absl::make_unique<TsdfMeshWritingPointsProcessor>(
      dictionary->GetString("filename"),
      mapping::CreateSubmapsOptions3D(dictionary->GetDictionary("submaps").get()),
      dictionary->HasKey("min_weight") ? dictionary->GetDouble("min_weight") : 0.0,
      mapping::CreateTSDFRangeDataInserterOptions3D(
          dictionary->GetDictionary("trajectory_builder_3d").get()),
      next);
}

TsdfMeshWritingPointsProcessor::TsdfMeshWritingPointsProcessor(std::basic_string<char> filename,
                                                               mapping::proto::SubmapsOptions3D options,
                                                               float min_weight,
                                                               const mapping::proto::TSDFRangeDataInserterOptions3D &range_data_inserter_3_d_options,
                                                               PointsProcessor *next)
    : next_(next),
      filename_(std::move(filename)),
      options_(std::move(options)),
      tsdf_range_data_inserter_3_d_(range_data_inserter_3_d_options),
      tsdf_(init_hybrid_grid_tsdf()),
      min_weight_(min_weight) {
}

mapping::HybridGridTSDF TsdfMeshWritingPointsProcessor::init_hybrid_grid_tsdf() {
  return mapping::HybridGridTSDF(options_.high_resolution(),
                                 options_.high_resolution_range_data_inserter_options()
                                     .tsdf_range_data_inserter_options_3d()
                                     .relative_truncation_distance(),
                                 options_.high_resolution_range_data_inserter_options()
                                     .tsdf_range_data_inserter_options_3d()
                                     .maximum_weight(),
                                 &conversion_tables_);
}

void TsdfMeshWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  tsdf_range_data_inserter_3_d_.Insert({batch->origin, batch->points, {}}, &tsdf_);

  next_->Process(std::move(batch));
}
PointsProcessor::FlushResult TsdfMeshWritingPointsProcessor::Flush() {
  size_t num_voxel = 0;
  for (auto &elem: tsdf_) {
    num_voxel++;
  }
  LOG(INFO) << "Created a TSDF map with " << num_voxel << " voxel";

  cartographer::mapping::MarchingCubes marching_cubes_handler;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PolygonMesh mesh;
  float isolevel = 0.0f;
  size_t triangle_count = 0;

  float resolution = tsdf_.resolution();

  for (auto it = ::cartographer::mapping::HybridGridTSDF::Iterator(tsdf_);
       !it.Done(); it.Next()) {
    const ::cartographer::mapping::TSDFVoxel voxel = it.GetValue();
    const float tsd = tsdf_.ValueConverter().ValueToTSD(voxel.discrete_tsd);
    const Eigen::Vector3f cell_center_global = tsdf_.GetCenterOfCell(it.GetCellIndex());

    if (voxel.discrete_weight <= min_weight_) {
      // Skip inner-object voxels
      continue;
    }

    cartographer::mapping::MarchingCubes::Cube cube;
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
      cube.tsd_weights[i] = tsdf_.GetWeight(cube.vertice_ids[i]);

      cube.tsd_values[i] =
          cube.tsd_weights[i] <= 0.0f ? tsd : tsdf_.GetTSD(cube.vertice_ids[i]);
    }
    triangle_count += marching_cubes_handler.ProcessCube(cube, cloud, isolevel);
  }

  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  cloud.clear();

  for (size_t i = 0; i < triangle_count; i++) {
    pcl::Vertices v;
    v.vertices.push_back(i * 3 + 0);
    v.vertices.push_back(i * 3 + 2);
    v.vertices.push_back(i * 3 + 1);
    mesh.polygons.push_back(v);
  }

  std::ofstream file(filename_ + ".ply", std::ofstream::out | std::ofstream::binary);
  marching_cubes_handler.WriteTSDFToPLYFile(file, mesh);
  file.close();
  LOG(INFO) << "Exported TSDF Mesh as PLY to "
            << boost::filesystem::complete(filename_ + ".ply").string();

  return PointsProcessor::FlushResult::kFinished;
}
}
}
