#include <cmath>
#include <fstream>
#include <string>

#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

// Classes to generate a cubic point cloud
#include "cartographer/evaluation/scan_cloud_generator.h"
#include "Eigen/Core"

// Classes to generate a TSDF
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/value_conversion_tables.h"
//#include "cartographer/mapping/3d/range_data_inserter_3d.h"
//#include "cartographer/mapping/3d/tsdf_range_data_inserter_3d.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"

#ifdef WITH_OPEN3D

#include "open3d/Open3D.h"

#endif

DEFINE_string(pointcloud_file, "",
              "File containing the point-cloud input.");

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFBuilder {

            // Generates a point cloud in shape of a cube and converts it into a shared pointer of an open3d point cloud.
            void generateCubicPointCloud(const std::shared_ptr<open3d::geometry::PointCloud> &cloud) {
                // Initialize an empty Point Cloud in the data format of cartographer
                cartographer::sensor::PointCloud cartographer_cloud;

                // Fill the Point Cloud with points (shape of a cube)
                cartographer::evaluation::ScanCloudGenerator myScanCloudGenerator(0.1);
                myScanCloudGenerator.generateCube(cartographer_cloud, 3.0, 0.0);

                // Convert the Point Cloud to a Vector of 3D-Eigen-Points.
                // This data format is more universal than the cartographer Point Cloud
                std::vector<Eigen::Vector3d> listOfPoints;
                for (cartographer::sensor::RangefinderPoint point : cartographer_cloud) {
                    listOfPoints.emplace_back(
                            Eigen::Vector3d{point.position.x(), point.position.y(), point.position.z()});
                }

                // Construct a Point Cloud in the data format of Open3D and fill it with the known points.
                open3d::geometry::PointCloud myPointCloud(listOfPoints);
                *cloud = myPointCloud;
            }

            static bool callbackFunction(open3d::visualization::Visualizer* visualizer) {
                std::cout << "This is a callback function" << std::endl;
                return true;
            }


            void drawTSDFs(std::shared_ptr<open3d::geometry::VoxelGrid> myVoxelGrid) {
                std::map<int, std::function< bool(open3d::visualization::Visualizer *)>> myMap;
                std::function<bool(open3d::visualization::Visualizer *)> myFunction = callbackFunction;
                myMap.insert(std::make_pair((int)'X', myFunction));
                open3d::visualization::DrawGeometriesWithKeyCallbacks({myVoxelGrid}, myMap);
            }

        public:
            void run(const std::string &point_cloud_filename) {

                std::shared_ptr<open3d::geometry::PointCloud> myPointCloudPointer =
                        std::make_shared<open3d::geometry::PointCloud>();

                // Generate a cloud in shape of a cube. Don't show the input from the .ply-file.
//                generateCubicPointCloud(myPointCloudPointer);

                // Read and show the input from the .ply-file.
                open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});

                myPointCloudPointer->EstimateNormals();
                myPointCloudPointer->OrientNormalsConsistentTangentPlane(10);
                open3d::visualization::DrawGeometries({myPointCloudPointer});







//            cartographer::mapping::ValueConversionTables myValueConversionTable;
//            cartographer::mapping::HybridGridTSDF myHybridGrid(0.1, 1.0, 10.0, &myValueConversionTable);
//            myHybridGrid.SetCell({0,5,10}, 0.9, 3.0);
//
//
//            // 1. Build a cartographer TSDF.
//            // 2. Convert it into an Open3D Voxel grid and display this.
//            // 3. Convert it into a ProtoBuf and output this.
//
//
            // A way to display TSDF Grids is to draw a Voxel Grid.
 //            std::shared_ptr<open3d::geometry::VoxelGrid> myVoxelGrid = open3d::geometry::VoxelGrid::CreateFromPointCloud(myPointCloud, 0.005);
 //            open3d::visualization::DrawGeometries({myVoxelGrid}, "Voxel Grid");

            // You can even color the voxels in a certain color
            if(!myPointCloudPointer->HasColors()) {
                for (Eigen::Vector3d p : myPointCloudPointer->points_) {
                    myPointCloudPointer->colors_.emplace_back(p);
                }
            }
            std::shared_ptr<open3d::geometry::VoxelGrid> myVoxelGrid = open3d::geometry::VoxelGrid::CreateFromPointCloud(*myPointCloudPointer, 0.01);
            drawTSDFs(myVoxelGrid);
//            open3d::visualization::DrawGeometries({myVoxelGrid}, "Voxel Grid");










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
        };
    }  // namespace mapping
}   // namespace cartographer
#endif

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pointcloud_file.empty()) {
        google::ShowUsageWithFlagsRestrict(argv[0], "pointcloud_converter");
        return EXIT_FAILURE;
    }
#ifdef WITH_OPEN3D
    cartographer::mapping::TSDFBuilder myTSDFBuilder;
    myTSDFBuilder.run(FLAGS_pointcloud_file);
#endif
}



// Possible Program Arguments
// -pointcloud_file "/home/leo/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply"
// -pointcloud_file "/home/leo/Downloads/bunny/reconstruction/bun_zipper_res4.ply"