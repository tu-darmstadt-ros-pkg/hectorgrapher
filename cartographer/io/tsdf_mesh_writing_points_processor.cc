//
// Created by bastian on 12.11.21.
//

#include "tsdf_mesh_writing_points_processor.h"

#include <utility>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "../mapping/3d/tsdf_range_data_inserter_3d.h"
#include "../mapping/3d/hybrid_grid_tsdf.h"
#include "../mapping/3d/submap_3d.h"

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
  for (auto &elem : tsdf_) {
    num_voxel++;
  }
  LOG(INFO) << "Created a TSDF map with " << num_voxel << " voxel";

  cartographer::mapping::MarchingCubes marching_cubes_handler;
  pcl::PolygonMesh mesh;
  const auto &all_submap_data = map_builder_->pose_graph()->GetAllSubmapData();
  const auto &robot_position = Eigen::Matrix<float, 3, 1>{NAN, NAN, NAN};

  marching_cubes_handler.ProcessTSDFMesh(mesh,
                                         all_submap_data,
                                         robot_position,
                                         kTsdfVisualizationHighRes,
                                         -1.0f,
                                         -1.0f,
                                         min_weight,
                                         true);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  std::ofstream file;
  file.open(filename + ".ply", std::ofstream::out | std::ofstream::binary);

  if (file.is_open()) {
    std::size_t cloud_size, polygon_size;
    const unsigned char count = 3;
    Eigen::Vector3f u, v, normal;
    u_char r, g, b;
    cloud_size = cloud.size();
    polygon_size = mesh.polygons.size();

    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "comment Created by Cartographer\n";
    file << "element vertex " << cloud_size << std::endl;
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "element face " << polygon_size << std::endl;
    file << "property list uchar uint vertex_indices\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    for (auto &p : cloud.points) {
      file.write(reinterpret_cast<const char *>(&(p.x)), sizeof(float));
      file.write(reinterpret_cast<const char *>(&(p.y)), sizeof(float));
      file.write(reinterpret_cast<const char *>(&(p.z)), sizeof(float));
    }
    for (auto &vertice_group : mesh.polygons) {
      // Write the number of elements
      file.write(reinterpret_cast<const char *>(&count), sizeof(unsigned char));
      file.write(reinterpret_cast<const char *>(&(vertice_group.vertices[0])), sizeof(uint32_t));
      file.write(reinterpret_cast<const char *>(&(vertice_group.vertices[1])), sizeof(uint32_t));
      file.write(reinterpret_cast<const char *>(&(vertice_group.vertices[2])), sizeof(uint32_t));
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
      file.write(reinterpret_cast<const char *>(&r), 1 * sizeof(u_char));
      file.write(reinterpret_cast<const char *>(&g), 1 * sizeof(u_char));
      file.write(reinterpret_cast<const char *>(&b), 1 * sizeof(u_char));
    }

    file.close();
    LOG(INFO) << "Exported TSDF Mesh as PLY to " << boost::filesystem::complete(filename + ".ply").string();

    return PointsProcessor::FlushResult::kFinished;
  }
  file.close();
  LOG(ERROR) << "Cannot write file to " << boost::filesystem::complete(filename + ".ply").string();

  return PointsProcessor::FlushResult::kFinished;
}
}
}