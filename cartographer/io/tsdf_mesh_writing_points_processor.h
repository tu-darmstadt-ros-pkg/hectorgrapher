//
// Created by bhirschel on 12.11.21.
//

#ifndef CARTOGRAPHER_CARTOGRAPHER_IO_TSDF_MESH_WRITING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_CARTOGRAPHER_IO_TSDF_MESH_WRITING_POINTS_PROCESSOR_H_

#include <cartographer/mapping/3d/hybrid_grid_tsdf.h>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/proto/3d/submaps_options_3d.pb.h"
#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"

namespace cartographer {
namespace io {

// Streams a PLY file to disk. The header is written in 'Flush'.
class TsdfMeshWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char *kConfigurationFileActionName = "write_tsdf_mesh";
  /**
   * Points Processor that constructs a TSDF map during the processing phase of the pipeline and
   * exports a surface mesh as ply file during flushing. All voxels below a TSD value of min_weight
   * are dropped.
   * @param file_writer cartographer file_writer handle getting initialized by the filename
   * dictionary entry
   * @param min_weight float specifying the minimum TSD value of voxels
   * @param use_high_res bool whether to use high-res or low-res tsdf specification.
   * RangeDataInserterOptions have to be adjusted accordingly
   * @param options as SubmapsOptions3D
   * @param range_data_inserter_3_d_options as mapping::proto::TSDFRangeDataInserterOptions3D
   * @param next pointer to the next points processor
   */
  TsdfMeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                 float min_weight,
                                 bool use_high_res,
                                 mapping::proto::SubmapsOptions3D options,
                                 const mapping::proto::TSDFRangeDataInserterOptions3D &range_data_inserter_3_d_options,
                                 PointsProcessor *next);

  static std::unique_ptr<TsdfMeshWritingPointsProcessor> FromDictionary(
      const FileWriterFactory &file_writer_factory,
      common::LuaParameterDictionary *dictionary,
      PointsProcessor *next);

  ~TsdfMeshWritingPointsProcessor() override {}

  TsdfMeshWritingPointsProcessor(const TsdfMeshWritingPointsProcessor &) = delete;
  TsdfMeshWritingPointsProcessor &operator=(const TsdfMeshWritingPointsProcessor &) =
  delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor *const next_;
  std::unique_ptr<FileWriter> file_writer_;
  const float min_weight_;
  const bool use_high_res_;
  const mapping::proto::SubmapsOptions3D options_;
  mapping::ValueConversionTables conversion_tables_;
  mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter_3_d_;
  mapping::HybridGridTSDF tsdf_;

  mapping::HybridGridTSDF init_hybrid_grid_tsdf();
};
}
}

#endif //CARTOGRAPHER_CARTOGRAPHER_IO_TSDF_MESH_WRITING_POINTS_PROCESSOR_H_
