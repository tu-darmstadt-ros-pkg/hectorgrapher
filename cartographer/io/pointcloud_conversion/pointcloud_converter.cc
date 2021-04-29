#include <cmath>
#include <fstream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

// Classes to generate a cubic point cloud
#include "cartographer/evaluation/scan_cloud_generator.h"
#include "Eigen/Core"

// Classes to generate a TSDF
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/value_conversion_tables.h"

// Classes for the lua dictionary
#include "cartographer/common/configuration_file_resolver.h"

// Classes for the ProtoBuffer
#include "cartographer/io/internal/mapping_state_serialization.cc"  //Todo: Das lässt uns CreateHeader() benutzen. Muss irgendwann weg!
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"

// Header file
#include "cartographer/io/pointcloud_conversion/tsdf_drawer.h"


#ifdef WITH_OPEN3D

#include "open3d/Open3D.h"

#endif

DEFINE_string(config_file, "", "LUA-file containing the configuration options");

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFBuilder {

            cartographer::common::LuaParameterDictionary *luaParameterDictionary;

            /**
             * Generate a point cloud in shape of a cube and convert it into a shared pointer of an open3d point cloud.
             *
             * The generation of the points is done by the method "generateCube"
             * by the class cartographer::evaluation::ScanCloudGenerator.
             *
             * Most of the method covers the convertion of the cartographer's representation of a point cloud
             * to the data format of open3d.
             *
             * @return open3d's representation of a point cloud inn shape of a cube.
             */
            std::shared_ptr<open3d::geometry::PointCloud> generateCubicPointCloud(
                    float cubeSideLength, float distanceBetweenPoints, float noise = 0.0) {
                // Initialize an empty Point Cloud in the data format of cartographer
                cartographer::sensor::PointCloud cartographer_cloud;

                // Fill the Point Cloud with points (shape of a cube)
                // Todo: Change scan_cloud_generator.cc to add noise on the z-dimension as well. Maybe an error?
                cartographer::evaluation::ScanCloudGenerator myScanCloudGenerator(distanceBetweenPoints);
                myScanCloudGenerator.generateCube(cartographer_cloud, cubeSideLength, noise);

                // Convert the Point Cloud to a Vector of 3D-Eigen-Points.
                // This data format is more universal than the cartographer Point Cloud
                std::vector<Eigen::Vector3d> listOfPoints;
                for (cartographer::sensor::RangefinderPoint point : cartographer_cloud) {
                    listOfPoints.emplace_back(
                            Eigen::Vector3d{point.position.x(), point.position.y(), point.position.z()});
                }

                // Construct a Point Cloud in the data format of Open3D and fill it with the known points.
                open3d::geometry::PointCloud myPointCloud(listOfPoints);
                return std::make_shared<open3d::geometry::PointCloud>(myPointCloud);
            }


            // Abgeschrieben von cartographer::mapping:3d::tsdf_range_data_inserter_3d
            // Sample TSDF Generation in Cartographer --> see evaluation/scan_matching_evaluation.cc
            // Todo: Gewichte werden nicht in die Rechnung einbezogen.
            //      Betragsmäßig kleinere Werte überschreiben betragsmäßig größere Werte.

            void raycastPointWithNormal(const Eigen::Vector3f &hit,
                                        const Eigen::Vector3f &normal,
                                        const float truncation_distance,
                                        HybridGridTSDF *tsdf) const {
                const Eigen::Vector3f ray_begin = hit - truncation_distance * normal;
                const Eigen::Vector3f ray_end = hit + truncation_distance * normal;

                const Eigen::Array3i begin_cell = tsdf->GetCellIndex(ray_begin);
                const Eigen::Array3i end_cell = tsdf->GetCellIndex(ray_end);
                const Eigen::Array3i delta = end_cell - begin_cell;

                CHECK(!(delta.x() == 0 && delta.y() == 0 && delta.z() == 0));

                const int num_samples = delta.cwiseAbs().maxCoeff();


                for (int position = 0; position <= num_samples; ++position) {
                    const Eigen::Array3i update_cell_index =
                            begin_cell +
                            (delta.cast<float>() * float(position) / float(num_samples)).round().cast<int>();

                    Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
                    float newTSD = (cell_center - hit).norm();
                    if ((cell_center - hit).dot(normal) < 0) {
                        newTSD = -newTSD;
                    }

                    // Todo: Welche Methode macht mehr Sinn? Ist die eine schneller als die andere?
                    // Only change the voxel TSD, if the new absolute value is lower than the old one.
                    if (!tsdf->IsKnown(update_cell_index) ||
                        std::abs(tsdf->GetTSD(update_cell_index)) > std::abs(newTSD)) {
//                        std::cout << update_cell_index << newTSD << std::endl;
                        tsdf->SetCell(update_cell_index, newTSD, 0.0f);
                    }

                    // Combine the new and old TSD by calculating the weighted average.
//                    float updatedWeight = 1.0f + tsdf->GetWeight(update_cell_index);
//                    float updatedTSD = (tsdf->GetTSD(update_cell_index) * tsdf->GetWeight(update_cell_index)
//                            + newTSD * 1.0f) / updatedWeight;
//                    tsdf->SetCell(update_cell_index, updatedTSD, updatedWeight);
                }
            }

            /**
             * Prepare the drawing of cartographer's HybridGridTSDF by converting it in open3d's VoxelGrid.
             *
             * The method colors the voxels blue for positive TSDs and red for negative TSDs.
             * Voxels with TSD near zero are colored black.
             *
             * @param hybridGrid pointer to a filled TSDF to be converted.
             * @param voxelSideLength edge length of the voxels of HybridGridTSDF.
             * @return a shared pointer to a VoxelGrid with origin (0,0,0).
             */
            std::shared_ptr<open3d::geometry::VoxelGrid> convertHybridGridToVoxelGrid(
                    cartographer::mapping::HybridGridTSDF *hybridGrid, float voxelSideLength, float maxTSD) {

                std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());
                voxelGrid->voxel_size_ = voxelSideLength;

                for (std::pair<Eigen::Array<int, 3, 1>, TSDFVoxel> nextVoxel : *hybridGrid) {
                    Eigen::Vector3i cellIndex = nextVoxel.first;

                    // Todo: Bessere, allgemeinere Farbcodierung, abhängig von der truncation distance anstatt "* 100"
                    Eigen::Vector3d color;
                    if (hybridGrid->GetTSD(cellIndex) > 0) {
                        color = {0.0, 0.0, hybridGrid->GetTSD(cellIndex) / maxTSD};
                    } else {
                        color = {hybridGrid->GetTSD(cellIndex) / -maxTSD, 0.0, 0.0};
                    }

//                    std::cout << cellIndex.x() << " " << cellIndex.y() << " " << cellIndex.z() << std::endl;
                    open3d::geometry::Voxel newVoxel = open3d::geometry::Voxel(cellIndex, color);
                    voxelGrid->AddVoxel(newVoxel);
                }
                return voxelGrid;
            }


        public:
            TSDFBuilder(const std::string &config_directory, const std::string &config_filename) {
                auto file_resolver =
                        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
                                std::vector<std::string>{config_directory});
                const std::string code = file_resolver->GetFileContentOrDie(config_filename);
                luaParameterDictionary = new cartographer::common::LuaParameterDictionary(code,
                                                                                          std::move(file_resolver));
            }


            void run() {

                cartographer::mapping::TSDFDrawer myTSDFDrawer;

                std::shared_ptr<open3d::geometry::PointCloud> myPointCloudPointer =
                        std::make_shared<open3d::geometry::PointCloud>();

                // Generate a cloud in shape of a cube. Don't show the input from the .ply-file.
                if (luaParameterDictionary->GetBool("generateCubicPointcloud")) {
                    myPointCloudPointer = generateCubicPointCloud(
                            (float) luaParameterDictionary->GetDouble("sidelengthCubicPointcloud"),
                            (float) luaParameterDictionary->GetDouble("distancePointsCubicPointcloud"),
                            (float) luaParameterDictionary->GetDouble("noiseCubicPointcloud"));
                } else {
                    // Read and show the input from the .ply-file.
                    std::string point_cloud_filename = luaParameterDictionary->GetString("pointcloudPath");
                    open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});
                    std::cout << "Loaded point cloud with " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if (luaParameterDictionary->GetBool("uniformDownSample")) {
                    int sampleRate = luaParameterDictionary->GetInt("sampleRateUniformDownSample");
                    myPointCloudPointer = myPointCloudPointer->UniformDownSample(sampleRate);
                    std::cout << "Uniform downsampling to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if (luaParameterDictionary->GetBool("voxelDownSample")) {
                    double voxelSize = luaParameterDictionary->GetDouble("voxelSizeVoxelDownSample");
                    myPointCloudPointer = myPointCloudPointer->VoxelDownSample(voxelSize);
                    std::cout << "Voxel downsampling to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if (luaParameterDictionary->GetBool("removeRadiusOutliers")) {
                    myPointCloudPointer = std::get<0>(myPointCloudPointer->RemoveRadiusOutliers(
                            luaParameterDictionary->GetInt("neighborsInSphereRadiusOutlier"),
                            luaParameterDictionary->GetDouble("neighborsInSphereRadiusOutlier")));
                    std::cout << "Removed outliers to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if (luaParameterDictionary->GetBool("cutRoofZAxis")) {
                    double cutoff = luaParameterDictionary->GetDouble("cutoffSize");
                    open3d::geometry::AxisAlignedBoundingBox myBoundingBox(myPointCloudPointer->GetMinBound(),
                                                                           myPointCloudPointer->GetMaxBound() -
                                                                           Eigen::Vector3d{0.0, 0.0, cutoff});
                    myPointCloudPointer = myPointCloudPointer->Crop(myBoundingBox);
                    std::cout << "Cropped point cloud to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }


                myPointCloudPointer->EstimateNormals();
                std::cout << "Estimated all normals for the point cloud." << std::endl;

                myPointCloudPointer->OrientNormalsConsistentTangentPlane(
                        luaParameterDictionary->GetInt("normalOrientationNearestNeighbours"));
                std::cout << "Oriented all normals by using the tangent plane." << std::endl;


                // Draw all points of the filtered point cloud
                open3d::visualization::DrawGeometries({myPointCloudPointer});


// #################################################################################################################
                // Show the VoxelGrid of the loaded point cloud


                // Apply colors to the points
                Eigen::Vector3d maxValues = myPointCloudPointer->GetMaxBound();
                Eigen::Vector3d minValues = myPointCloudPointer->GetMinBound();
                Eigen::Vector3d ranges = maxValues - minValues;
                if (!myPointCloudPointer->HasColors()) {
                    for (const Eigen::Vector3d &p : myPointCloudPointer->points_) {
                        myPointCloudPointer->colors_.emplace_back(
                                Eigen::Vector3d{(p.x() - minValues.x()) * 1.0 / ranges.x(), 0.0, 0.0});
                    }
                }

                float gridVoxelSideLength = (float) luaParameterDictionary->GetDouble("absoluteVoxelSize");
                int numberOfVoxels = (int) (ranges.x() * ranges.y() * ranges.z() / pow(gridVoxelSideLength, 3));
                std::cout << "Created VoxelGrid with " << numberOfVoxels << " possible voxels." << std::endl;

                std::shared_ptr<open3d::geometry::VoxelGrid> pclVoxelGridPointer =
                        open3d::geometry::VoxelGrid::CreateFromPointCloud(*myPointCloudPointer, gridVoxelSideLength);

                myTSDFDrawer.drawTSDF(pclVoxelGridPointer);



// #################################################################################################################
                // Build a HybridGridTSDF = cartographer's representation of a TSDF


                float absoluteTruncationDistance =
                        (float) luaParameterDictionary->GetDouble("absoluteTruncationDistance");
                float maxWeight = (float) luaParameterDictionary->GetDouble(
                        "maxTSDFWeight");  // Todo: Was sollte das maximale Gewicht sein?
                float relativeTruncationDistance = absoluteTruncationDistance / gridVoxelSideLength;

                cartographer::mapping::ValueConversionTables myValueConversionTable;
                cartographer::mapping::HybridGridTSDF myHybridGridTSDF(
                        gridVoxelSideLength, relativeTruncationDistance, maxWeight, &myValueConversionTable);


                // Build the TSDF by raytracing every point/normal pair from the point cloud
                for (long unsigned int i = 0; i < myPointCloudPointer->points_.size(); i++) {
                    raycastPointWithNormal(
                            myPointCloudPointer->points_.at(i).cast<float>(),
                            myPointCloudPointer->normals_.at(i).cast<float>(),
                            absoluteTruncationDistance,
                            &myHybridGridTSDF);
                }


                std::cout << "Press the left/right keys to slice through the voxel grid!" << std::endl;
                std::cout << "Press the key >o< to change the slicing orientation." << std::endl;

                // Show the VoxelGrid of the TSDF
                std::shared_ptr<open3d::geometry::VoxelGrid> tsdfVoxelGridPointer = convertHybridGridToVoxelGrid(
                        &myHybridGridTSDF, gridVoxelSideLength, absoluteTruncationDistance);
                myTSDFDrawer.drawTSDF(tsdfVoxelGridPointer);


// #################################################################################################################
                // Save some slices as png

                myTSDFDrawer.saveSliceAsPNG(0.0, "../cartographer/io/pointcloud_conversion/images/testimage.png");


// #################################################################################################################
                // Build a ProtoBuffer
                cartographer::io::ProtoStreamWriter writer(
                        "../cartographer/io/pointcloud_conversion/ProtoBuffers/TSDFProtoBufferTest.pbstream");

                // Das kann bald weg hoffentlich v
//                mapping::proto::SerializationHeader myHeader = cartographer::io::CreateHeader();
//                writer.WriteProto(myHeader);
                // Das hier ^

                const std::string kMapBuilderLua = R"text(
                    include "map_builder.lua"
                    MAP_BUILDER.use_trajectory_builder_2d = true
                    MAP_BUILDER.pose_graph.optimize_every_n_nodes = 0
                    MAP_BUILDER.pose_graph.global_sampling_ratio = 0.05
                    MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 0
                    return MAP_BUILDER)text";

                auto file_resolver =
                        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
                                std::vector<std::string>{"/home/leo/hector/src/cartographer/configuration_files"});
                cartographer::common::LuaParameterDictionary poseGraphDict(kMapBuilderLua, std::move(file_resolver));

                cartographer::mapping::proto::MapBuilderOptions options;
                options = cartographer::mapping::CreateMapBuilderOptions(&poseGraphDict);

                cartographer::common::ThreadPool thread_pool(options.num_background_threads());

                std::unique_ptr<cartographer::mapping::PoseGraph> posegraph =
                        absl::make_unique<cartographer::mapping::PoseGraph3D>(
                                options.pose_graph_options(),
                                absl::make_unique<cartographer::mapping::optimization::OptimizationProblem3D>(
                                        options.pose_graph_options().optimization_problem_options()),
                                &thread_pool);

//                writer.WriteProto(cartographer::io::SerializePoseGraph(*posegraph, false));



                std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> all_trajectory_builder_options;

                // This has to hold (will be tested in deserialzation!)
//                CHECK_EQ(
//                        cartographer::io::SerializePoseGraph(*posegraph, false).pose_graph().trajectory_size(),
//                        cartographer::io::SerializeTrajectoryBuilderOptions(
//                                all_trajectory_builder_options,
//                                cartographer::io::GetValidTrajectoryIds(posegraph->GetTrajectoryStates())
//                        ).all_trajectory_builder_options().options_with_sensor_ids_size());


                cartographer::io::WritePbStream(*posegraph, all_trajectory_builder_options, &writer, false);

                proto::HybridGridTSDF myProtoTSDF = myHybridGridTSDF.ToProto();
                writer.WriteProto(myProtoTSDF);

                writer.Close();
            }

        };
    }  // namespace mapping
}   // namespace cartographer
#endif

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_config_file.empty()) {
        google::ShowUsageWithFlagsRestrict(argv[0], "pointcloud_converter");
        return EXIT_FAILURE;
    }
#ifdef WITH_OPEN3D
    cartographer::mapping::TSDFBuilder myTSDFBuilder(
            "../cartographer/io/pointcloud_conversion/configurations",
            FLAGS_config_file);
    myTSDFBuilder.run();
#endif
}
