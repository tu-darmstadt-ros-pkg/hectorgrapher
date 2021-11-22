//
// Created by bastian on 12.11.21.
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
  TsdfMeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                 mapping::proto::SubmapsOptions3D options,
                                 float min_weight,
                                 const mapping::proto::RangeDataInserterOptions3D& range_data_inserter_3_d_options,
                                 PointsProcessor *const next);

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
  std::unique_ptr<FileWriter> file_;
  const mapping::proto::SubmapsOptions3D options_;
  mapping::ValueConversionTables *conversion_tables_;
  mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter_3_d_;
  mapping::HybridGridTSDF tsdf_;
  const float min_weight_;

  mapping::HybridGridTSDF init();
};
}
}

#endif //CARTOGRAPHER_CARTOGRAPHER_IO_TSDF_MESH_WRITING_POINTS_PROCESSOR_H_
