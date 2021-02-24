#include <cmath>
#include <fstream>
#include <string>

#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/3d/range_data_inserter_3d.h"
#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"

#ifdef WITH_OPEN3D
#include "open3d/Open3D.h"
#endif

DEFINE_string(pointcloud_file,"",
              "File containing the point-cloud input.");

#ifdef WITH_OPEN3D
namespace cartographer {
namespace mapping {
  void Run(const std::string& point_cloud_filename) {
    std::shared_ptr<open3d::geometry::PointCloud> cloud =
        std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud(point_cloud_filename, *cloud,{"auto", true, true, true});
    open3d::visualization::DrawGeometries({cloud});

//    Sample TSDF Generation in Cartographer --> see evaluation/scan_matching_evaluation.cc
//    cartographer::mapping::proto::RangeDataInserterOptions3D
//        range_data_inserter_options;
//    auto parameter_dictionary_range_data_inserter = common::MakeDictionary(R"text(
//        return {
//      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_3D",
//      probability_grid_range_data_inserter = {
//        hit_probability = 0.55,
//        miss_probability = 0.49,
//        num_free_space_voxels = 2,
//      },
//      tsdf_range_data_inserter = {
//        relative_truncation_distance = 4,
//        maximum_weight = 1000.,
//        num_free_space_voxels = 0,
//        project_sdf_distance_to_scan_normal = true,
//        weight_function_epsilon = 1,
//        weight_function_sigma = 4.,
//        normal_estimate_max_nn = 20.,
//        normal_estimate_radius = 0.3,
//      },})text");
//    range_data_inserter_options =
//        cartographer::mapping::CreateRangeDataInserterOptions3D(
//            parameter_dictionary_range_data_inserter.get());
//
//    mapping::ValueConversionTables conversion_tables_;
//    mapping::HybridGridTSDF hybrid_grid_tsdf(
//        0.05,
//        range_data_inserter_options.tsdf_range_data_inserter_options_3d()
//            .relative_truncation_distance(),
//        range_data_inserter_options.tsdf_range_data_inserter_options_3d()
//            .maximum_weight(),
//        &conversion_tables_);
//    mapping::TSDFRangeDataInserter3D tsdf_range_data_inserter(
//        range_data_inserter_options);
//    tsdf_range_data_inserter.Insert(sample.range_data, &hybrid_grid_tsdf);

  }
}  // namespace mapping
}  // namespace cartographer

#endif

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pointcloud_file.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pointcloud_converter");
    return EXIT_FAILURE;
  }
#ifdef WITH_OPEN3D
  cartographer::mapping::Run(FLAGS_pointcloud_file);
#endif
}