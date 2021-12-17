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
    const FileWriterFactory& file_writer_factory,
    common::LuaParameterDictionary *const dictionary,
    PointsProcessor *const next) {
  return absl::make_unique<TsdfMeshWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename") + ".ply"),
      mapping::CreateSubmapsOptions3D(dictionary->GetDictionary("submaps").get()),
      dictionary->HasKey("min_weight") ? dictionary->GetDouble("min_weight") : 0.0,
      mapping::CreateTSDFRangeDataInserterOptions3D(
          dictionary->GetDictionary("trajectory_builder_3d").get()),
      next);
}

TsdfMeshWritingPointsProcessor::TsdfMeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                                               mapping::proto::SubmapsOptions3D options,
                                                               float min_weight,
                                                               const mapping::proto::TSDFRangeDataInserterOptions3D &range_data_inserter_3_d_options,
                                                               PointsProcessor *next)
    : next_(next),
      file_writer_(std::move(file_writer)),
      options_(std::move(options)),
      tsdf_range_data_inserter_3_d_(range_data_inserter_3_d_options),
      tsdf_(init_hybrid_grid_tsdf()),
      min_weight_(min_weight) {
}

mapping::HybridGridTSDF TsdfMeshWritingPointsProcessor::init_hybrid_grid_tsdf() {
  return mapping::HybridGridTSDF(static_cast<float>(options_.high_resolution()),
                                 static_cast<float>(options_
                                     .high_resolution_range_data_inserter_options()
                                     .tsdf_range_data_inserter_options_3d()
                                     .relative_truncation_distance()),
                                 static_cast<float>(options_
                                     .high_resolution_range_data_inserter_options()
                                     .tsdf_range_data_inserter_options_3d()
                                     .maximum_weight()),
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

//    LOG(INFO) << "TSD: " << tsd << " in [" << tsdf_.ValueConverter().getMinTSD() << ", " << tsdf_.ValueConverter().getMaxTSD() << "]";
//    if (tsd <= 0.0f) {
//      // Skip inner-object voxels with TSD < 0 with TSD in [min_tsd, max_tsd], eg [-0.25, 0.25]
//      continue;
//    }

//    LOG(INFO) << "Weight: " << tsdf_.ValueConverter().ValueToWeight(voxel.discrete_weight) << " in [" << tsdf_.ValueConverter().getMinWeight() << ", " << tsdf_.ValueConverter().getMaxWeight() << "]";
    if (tsdf_.ValueConverter().ValueToWeight(voxel.discrete_weight) <= min_weight_) {
      // Skip voxels with low weight with weight in [min_weight = 0.0, max_weight], eg [0.0, 1000.0]
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
          (cube.tsd_weights[i] <= 0.0f) ? tsd : tsdf_.GetTSD(cube.vertice_ids[i]);
    }
    triangle_count += cartographer::mapping::MarchingCubes::ProcessCube(cube, cloud, isolevel);
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

//  std::ofstream file(filename_ + ".ply", std::ofstream::out | std::ofstream::binary);
//  cartographer::mapping::MarchingCubes::WriteTSDFToPLYFile(file, mesh);
//  file.close();

  std::stringstream stream;
  cartographer::mapping::MarchingCubes::WriteTSDFToStringstream(stream, mesh);
  stream.seekg(0, std::ios::end);
  file_writer_->Write(stream.str().data(), stream.tellg());
  file_writer_->Close();
//  LOG(INFO) << "Exported TSDF Mesh as PLY to "
//            << boost::filesystem::complete(filename_ + ".ply").string();

  return PointsProcessor::FlushResult::kFinished;
}
}
}
