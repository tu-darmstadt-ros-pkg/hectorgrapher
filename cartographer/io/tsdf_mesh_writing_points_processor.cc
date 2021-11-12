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
  return absl::make_unique<TsdfMeshWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")),
      mapping::CreateSubmapsOptions3D(dictionary->GetDictionary("submaps").get()),
      dictionary->HasKey("min_weight") ? dictionary->GetDouble("min_weight") : 0.0,
      next);
}

TsdfMeshWritingPointsProcessor::TsdfMeshWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                                               mapping::proto::SubmapsOptions3D options,
                                                               float min_weight,
                                                               PointsProcessor *const next)
    : next_(next),
      file_(std::move(file_writer)),
      options_(std::move(options)),
      min_weight_(min_weight),
      tsdf_(init()){
  init();
}

mapping::HybridGridTSDF TsdfMeshWritingPointsProcessor::init() {
  return mapping::HybridGridTSDF(options_.high_resolution(),
                                       options_.high_resolution_range_data_inserter_options()
                                           .tsdf_range_data_inserter_options_3d()
                                           .relative_truncation_distance(),
                                       options_.high_resolution_range_data_inserter_options()
                                           .tsdf_range_data_inserter_options_3d()
                                           .maximum_weight(),
                                       conversion_tables_);
}

void TsdfMeshWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  //TODO need to check for option WITH_PCL


  next_->Process(std::move(batch));
}
PointsProcessor::FlushResult TsdfMeshWritingPointsProcessor::Flush() {
  return PointsProcessor::FlushResult::kFinished;
}
}
}