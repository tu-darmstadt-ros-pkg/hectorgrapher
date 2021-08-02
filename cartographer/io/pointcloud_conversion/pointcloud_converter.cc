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
#include "cartographer/io/internal/mapping_state_serialization.cc"
#include "cartographer/io/proto_stream.h"
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
const std::string path_to_home = getenv("HOME");


#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFBuilder {

            cartographer::common::LuaParameterDictionary *luaParameterDictionary;
            std::string configuration_name;

            /**
             * Generate a point cloud in shape of a cube and convert it into a shared pointer of an open3d point cloud.
             *
             * The generation of the points is done by the method "generateCube"
             * by the class cartographer::evaluation::ScanCloudGenerator.
             *
             * Most of the method covers the convertion of the cartographer's representation of a point cloud
             * to the data format of open3d.
             *
             * @return open3d's representation of a point cloud in shape of a cube.
             */
            static std::shared_ptr<open3d::geometry::PointCloud> generateCubicPointCloud(
                    float cubeSideLength, float distanceBetweenPoints, float noise = 0.0) {
                // Initialize an empty Point Cloud in the data format of cartographer
                cartographer::sensor::PointCloud cartographer_cloud;

                // Fill the Point Cloud with points (shape of a cube)
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

            static void raycastPointWithNormal(const Eigen::Vector3f &hit,
                                               const Eigen::Vector3f &normal,
                                               const float truncation_distance,
                                               HybridGridTSDF *tsdf) {
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

                    // Only change the voxel TSD, if the new absolute value is lower than the old one.
//                    if (!tsdf->IsKnown(update_cell_index) ||
//                        std::abs(tsdf->GetTSD(update_cell_index)) > std::abs(newTSD)) {
//                        tsdf->SetCell(update_cell_index, newTSD, 0.0f);
//                    }

                    // Combine the new and old TSD by calculating the weighted average.
                    float updatedWeight = 1.0f + tsdf->GetWeight(update_cell_index);
                    float updatedTSD = (tsdf->GetTSD(update_cell_index) * tsdf->GetWeight(update_cell_index)
                                        + newTSD * 1.0f) / updatedWeight;
                    tsdf->SetCell(update_cell_index, updatedTSD, updatedWeight);
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
            static std::shared_ptr<open3d::geometry::VoxelGrid> convertHybridGridToVoxelGrid(
                    cartographer::mapping::HybridGridTSDF *hybridGrid, float voxelSideLength, float maxTSD) {

                std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());
                voxelGrid->voxel_size_ = voxelSideLength;

                for (std::pair<Eigen::Array<int, 3, 1>, TSDFVoxel> nextVoxel : *hybridGrid) {
                    Eigen::Vector3i cellIndex = nextVoxel.first;

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
                configuration_name = config_filename;
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
                    std::string point_cloud_filename =
                            path_to_home + luaParameterDictionary->GetString("pointcloudPath");
                    open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});
                    std::cout << "Loaded point cloud with exactly " << myPointCloudPointer->points_.size() << " points."
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
                            luaParameterDictionary->GetDouble("sphereSizeRadiusOutliers")));
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
//                open3d::visualization::DrawGeometries({myPointCloudPointer});


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

                //             myTSDFDrawer.drawTSDF(pclVoxelGridPointer);



// #################################################################################################################
                // Build a HybridGridTSDF = cartographer's representation of a TSDF


                float absoluteTruncationDistance =
                        (float) luaParameterDictionary->GetDouble("absoluteTruncationDistance");
                float maxWeight = 100.0;
                float relativeTruncationDistance = absoluteTruncationDistance / gridVoxelSideLength;

                cartographer::mapping::ValueConversionTables myValueConversionTable;
                std::unique_ptr<GridInterface> myHybridGridTSDF = absl::make_unique<HybridGridTSDF>(
                        gridVoxelSideLength, relativeTruncationDistance, maxWeight, &myValueConversionTable);


                // Build the TSDF by raytracing every point/normal pair from the point cloud
                for (long unsigned int i = 0; i < myPointCloudPointer->points_.size(); i++) {
                    raycastPointWithNormal(
                            myPointCloudPointer->points_.at(i).cast<float>(),
                            myPointCloudPointer->normals_.at(i).cast<float>(),
                            absoluteTruncationDistance,
                            dynamic_cast<HybridGridTSDF *>(myHybridGridTSDF.get()));
                }


                // std::cout << "Press the left/right keys to slice through the voxel grid!" << std::endl;
                // std::cout << "Press the key >o< to change the slicing orientation." << std::endl;

                // Show the VoxelGrid of the TSDF
                std::shared_ptr<open3d::geometry::VoxelGrid> tsdfVoxelGridPointer = convertHybridGridToVoxelGrid(
                        dynamic_cast<HybridGridTSDF *>(myHybridGridTSDF.get()), gridVoxelSideLength,
                        absoluteTruncationDistance);
//                  myTSDFDrawer.drawTSDF(tsdfVoxelGridPointer);


// #################################################################################################################
                // Save some slices as png
                std::string imgfilename;
                imgfilename = path_to_home +
                              "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                              + configuration_name + "_img_x.png";
                myTSDFDrawer.saveSliceAsPNG(0, 0, imgfilename.c_str(), tsdfVoxelGridPointer);

                imgfilename = path_to_home +
                              "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                              + configuration_name + "_img_y.png";
                myTSDFDrawer.saveSliceAsPNG(0, 1, imgfilename.c_str(), tsdfVoxelGridPointer);

                for (int i = 0; i < 6; i++) {
                    imgfilename = path_to_home +
                                  "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                                  + configuration_name + "_img_z" + std::to_string(i) + ".png";

                    myTSDFDrawer.saveSliceAsPNG(luaParameterDictionary->GetInt("imageSliceIndex") + 3 * i, 2,
                                                imgfilename.c_str(),
                                                tsdfVoxelGridPointer);
                }




// #################################################################################################################
                // Build a ProtoBuffer

                // --- Build a submap for the TSDF ---
                const cartographer::transform::Rigid3<double> my_transform;
                const Eigen::VectorXf rot_sm_histo(1);
                cartographer::mapping::ValueConversionTables my_vct;
                const cartographer::common::Time my_time = cartographer::common::FromUniversal(100);

                cartographer::mapping::Submap3D my_submap(
                        my_transform,
                        std::unique_ptr<GridInterface>(static_cast<GridInterface *>(myHybridGridTSDF.release())),
                        std::unique_ptr<GridInterface>(static_cast<GridInterface *>(myHybridGridTSDF.release())),
                        rot_sm_histo, &my_vct, my_time);

                // --- Build a pose graph for the submap (with some dummy values) ---
                proto::PoseGraphOptions my_posegraph_options;
                const optimization::proto::OptimizationProblemOptions my_opt_prob_options;
                std::unique_ptr<optimization::OptimizationProblem3D> my_opt_prob =
                        absl::make_unique<optimization::OptimizationProblem3D>(my_opt_prob_options);
                cartographer::common::ThreadPool my_thread_pool(1);

                my_posegraph_options.set_global_sampling_ratio(0.1);
                my_posegraph_options.mutable_constraint_builder_options()->set_sampling_ratio(0.1);

                cartographer::mapping::PoseGraph3D posegraph(
                        my_posegraph_options,
                        std::move(my_opt_prob),
                        &my_thread_pool
                );

                // --- Put the submap in the pose graph ---
                posegraph.AddSubmapFromProto(my_transform, my_submap.ToProto(false));

                // --- Write the pose graph as protoBuffer ---
                proto::TrajectoryBuilderOptionsWithSensorIds my_traj_builder_options;
                std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> trajectory_builder_options;
                trajectory_builder_options.push_back(my_traj_builder_options);

                cartographer::io::ProtoStreamWriter writer(
                        path_to_home +
                        "/hector/src/cartographer/cartographer/io/pointcloud_conversion/" +
                        "ProtoBuffers/TSDFProtoBufferTest.pbstream");

                cartographer::io::WritePbStream(posegraph, trajectory_builder_options, &writer, false);
                writer.Close();

                // --- Run some tests to make sure we did everything correctly ---
                CHECK_EQ(
                        cartographer::io::SerializePoseGraph(posegraph, false).pose_graph().trajectory_size(),
                        cartographer::io::SerializeTrajectoryBuilderOptions(
                                trajectory_builder_options,
                                cartographer::io::GetValidTrajectoryIds(posegraph.GetTrajectoryStates())
                        ).all_trajectory_builder_options().options_with_sensor_ids_size());
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
            path_to_home + "/hector/src/cartographer/cartographer/io/pointcloud_conversion/configurations",
            FLAGS_config_file);
    myTSDFBuilder.run();
#endif
}
