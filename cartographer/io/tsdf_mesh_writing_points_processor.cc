//
// Created by bastian on 12.11.21.
//

#include "tsdf_mesh_writing_points_processor.h"

#include <utility>
#include <pcl/PolygonMesh.h>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "../mapping/3d/tsdf_range_data_inserter_3d.h"
#include "../mapping/3d/hybrid_grid_tsdf.h"
#include "../mapping/3d/submap_3d.h"
#include "cartographer/mapping/marching_cubes.h"

namespace cartographer {
namespace io {
std::unique_ptr<TsdfMeshWritingPointsProcessor>
TsdfMeshWritingPointsProcessor::FromDictionary(
    const FileWriterFactory &file_writer_factory,
    common::LuaParameterDictionary *const dictionary,
    PointsProcessor *const next) {
//  const std::string& code("");
//  common::LuaParameterDictionary trajectory_dictionary("", std::make_unique<common::ConfigurationFileResolver>(common::ConfigurationFileResolver(std::vector<std::string>())));
  return absl::make_unique<TsdfMeshWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")),
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
                                                               PointsProcessor *const next)
    : next_(next),
      file_(std::move(file_writer)),
      options_(std::move(options)),
      tsdf_range_data_inserter_3_d_(range_data_inserter_3_d_options),
      tsdf_(init()),
      min_weight_(min_weight) {
}

mapping::HybridGridTSDF TsdfMeshWritingPointsProcessor::init() {
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
  size_t point_count = 0;

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
    point_count += marching_cubes_handler.ProcessCube(cube, cloud, isolevel);
  }

  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

//  std::ofstream file;
//  file.open(file_.get()->GetFilename() + ".ply", std::ofstream::out | std::ofstream::binary);

  std::size_t cloud_size, polygon_size;
  const unsigned char count = 3;
  Eigen::Vector3f u, v, normal;
  u_char r, g, b;
  cloud_size = cloud.size();
  polygon_size = mesh.polygons.size();

  file_->Write("ply\n", 4);
  file_->Write("format binary_little_endian 1.0\n", 32);
  file_->Write("comment Created by Cartographer\n", 32);
  file_->Write("element vertex ", 15);
  file_->Write(reinterpret_cast<const char *>(&cloud_size), sizeof(std::size_t));
  file_->Write("\n", 1);
  file_->Write("property float x\n", 17);
  file_->Write("property float y\n", 17);
  file_->Write("property float z\n", 17);
  file_->Write("element face ", 13);
  file_->Write(reinterpret_cast<const char *>(&polygon_size), sizeof(std::size_t));
  file_->Write("\n", 1);
  file_->Write("property list uchar uint vertex_indices\n", 40);
  file_->Write("property uchar red\n", 19);
  file_->Write("property uchar green\n", 21);
  file_->Write("property uchar blue\n", 20);
  file_->Write("end_header\n", 11);

  for (auto &p: cloud.points) {
    file_->Write(reinterpret_cast<const char *>(&(p.x)), sizeof(float));
    file_->Write(reinterpret_cast<const char *>(&(p.y)), sizeof(float));
    file_->Write(reinterpret_cast<const char *>(&(p.z)), sizeof(float));
  }
  for (auto &vertice_group: mesh.polygons) {
    // Write the number of elements
    file_->Write(reinterpret_cast<const char *>(&count), sizeof(unsigned char));
    file_->Write(reinterpret_cast<const char *>(&(vertice_group.vertices[0])), sizeof(uint32_t));
    file_->Write(reinterpret_cast<const char *>(&(vertice_group.vertices[1])), sizeof(uint32_t));
    file_->Write(reinterpret_cast<const char *>(&(vertice_group.vertices[2])), sizeof(uint32_t));
    // Write the colors
    u = {cloud[vertice_group.vertices[1]].x - cloud[vertice_group.vertices[0]].x,
         cloud[vertice_group.vertices[1]].y - cloud[vertice_group.vertices[0]].y,
         cloud[vertice_group.vertices[1]].z - cloud[vertice_group.vertices[0]].z};
    v = {cloud[vertice_group.vertices[2]].x - cloud[vertice_group.vertices[0]].x,
         cloud[vertice_group.vertices[2]].y - cloud[vertice_group.vertices[0]].y,
         cloud[vertice_group.vertices[2]].z - cloud[vertice_group.vertices[0]].z};
    normal = u.cross(v).normalized();
    r = static_cast<u_char>((normal.x() + 1.0f) * 0.5f * 255);
    g = static_cast<u_char>((normal.y() + 1.0f) * 0.5f * 255);
    b = static_cast<u_char>((normal.z() + 1.0f) * 0.5f * 255);
    file_->Write(reinterpret_cast<const char *>(&r), 1 * sizeof(u_char));
    file_->Write(reinterpret_cast<const char *>(&g), 1 * sizeof(u_char));
    file_->Write(reinterpret_cast<const char *>(&b), 1 * sizeof(u_char));
  }

//  LOG(ERROR) << "Cannot write file to " << boost::filesystem::complete(filename + ".ply").string();

  return PointsProcessor::FlushResult::kFinished;
}
}
}
