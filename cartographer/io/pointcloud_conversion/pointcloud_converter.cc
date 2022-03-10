#include <cmath>
#include <fstream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

// Classes to generate a cubic point cloud
#include "Eigen/Core"

// Classes to generate a TSDF
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/value_conversion_tables.h"

// Classes for the lua dictionary
#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"

// Classes for the ProtoBuffer
#include "cartographer/io/internal/mapping_state_serialization.cc"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"


// Header file
#include "cartographer/io/pointcloud_conversion/tsdf_drawer.h"
#include "cartographer/io/pointcloud_conversion/pointcloud_creator.h"


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
            static std::shared_ptr<open3d::geometry::VoxelGrid> convertTSDFToVoxelGrid(
                    cartographer::mapping::HybridGridTSDF *hybridGrid, float voxelSideLength, float maxTSD) {

                std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());
                voxelGrid->voxel_size_ = voxelSideLength;

                for (std::pair<Eigen::Array<int, 3, 1>, TSDFVoxel> nextVoxel: *hybridGrid) {
                    Eigen::Vector3i cellIndex = nextVoxel.first;

                    Eigen::Vector3d color;
                    if (hybridGrid->GetTSD(cellIndex) > 0) {
                        color = {0.0, 0.0, hybridGrid->GetTSD(cellIndex) / maxTSD};
                    } else {
                        color = {hybridGrid->GetTSD(cellIndex) / -maxTSD, 0.0, 0.0};
                    }
                    open3d::geometry::Voxel newVoxel = open3d::geometry::Voxel(cellIndex, color);
                    voxelGrid->AddVoxel(newVoxel);
                }
                return voxelGrid;
            }

            /**
             * Prepare the drawing of cartographer's HybridGrid by converting it in open3d's VoxelGrid.
             *
             * The method colors the occupied voxels blue.
             *
             * @param hybridGrid pointer to a filled occupancy grid to be converted.
             * @param voxelSideLength edge length of the voxels of HybridGrid.
             * @return a shared pointer to a VoxelGrid with origin (0,0,0).
             */
            static std::shared_ptr<open3d::geometry::VoxelGrid> convertOccupancyGridToVoxelGrid(
                    cartographer::mapping::HybridGrid *hybridGrid, float voxelSideLength) {
                std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());
                voxelGrid->voxel_size_ = voxelSideLength;
                for (auto nextVoxel: *hybridGrid) {
                    Eigen::Vector3i cellIndex = nextVoxel.first;
                    Eigen::Vector3d color = {0.0, 0.0, hybridGrid->GetProbability(cellIndex)};

                    open3d::geometry::Voxel newVoxel = open3d::geometry::Voxel(cellIndex, color);
                    voxelGrid->AddVoxel(newVoxel);
                }
                return voxelGrid;
            }

            /**
            * Finds a good division for the original grid.
            *
            * The method uses the given dimensions of a hybridGrid to find the most reasonable division.
            * The division should be as close to a cube as possible, while also consisting of a maximal number of subgrids.
            *
            * @param numberOfSubmaps how many submaps should be created from the full grid.
            * @param gridSize the size of the original HybridGrid
            * @return the fragmentation dimension
            */
            static Eigen::Vector3i getFragmentationForHybridGrid(int numberOfSubmaps, Eigen::Vector3i gridSize) {
                Eigen::Vector3i fragmentation = Eigen::Vector3i::Zero();

                if (gridSize.x() > gridSize.y()) {
                    double relation = (double) gridSize.x() / gridSize.y();
                    fragmentation.x() = (int) ((relation + 1) / 2 + std::sqrt(
                            std::pow((relation + 1) / 2, 2) - (1 - numberOfSubmaps) * relation));
                    fragmentation.y() = numberOfSubmaps / (fragmentation.x() - 1) + 1;
                    if (fragmentation.y() < 2) {
                        fragmentation.y() = 2;
                        fragmentation.x() = numberOfSubmaps + 1;
                    }
                } else {
                    double relation = (double) gridSize.y() / gridSize.x();
                    fragmentation.y() = (int) ((relation + 1) / 2 + std::sqrt(
                            std::pow((relation + 1) / 2, 2) - (1 - numberOfSubmaps) * relation));
                    fragmentation.x() = numberOfSubmaps / (fragmentation.y() - 1) + 1;
                    if (fragmentation.x() < 2) {
                        fragmentation.x() = 2;
                        fragmentation.y() = numberOfSubmaps + 1;
                    }
                }
                fragmentation.z() = 2;

                std::cout << "For " << numberOfSubmaps << " given submaps and a grid size of ";
                std::cout << gridSize.x() << " x " << gridSize.y() << " x " << gridSize.z() << ", ";
                std::cout << "we created fragments in the shape " << fragmentation.x() << " x " << fragmentation.y()
                          << " x " << fragmentation.z() << "." << std::endl;

                return fragmentation;
            }

            /**
            * Calculates the index of the subgrid for a given voxel.
            *
            * The grid is expected to be approximated with a cuboid and divided into subgrids in the shape of cuboids.
            * The subgrids are NOT ordered like an array, but instead along a path through the grid.
            * |__0__|__5__|__6__|
            * |__1__|__4__|__7__|
            * |__2__|__3__|__8__|
            *
            * @param voxel the index of the voxel to be inserted in a subgrid.
            * @param gridSize the dimensions of the original HybridGrid
            * @param fragmentation the dimensions of the division of the original grid into smaller fragments.
            * @param variations Matrix containing 8 vectors who count from 000 to 111
            * @return a list of submap indices for the given voxel
            */
            static std::vector<int> getSubmapIndicesForVoxel(const Eigen::Vector3i &voxel,
                                                             const Eigen::Vector3i &gridSize,
                                                             const Eigen::Vector3i &fragmentation,
                                                             const Eigen::Matrix<int, 3, 8> &variations) {
                Eigen::Vector3i fragment = voxel.array() * fragmentation.array();
                fragment = fragment.array() / (gridSize + Eigen::Vector3i::Ones()).array();

                std::vector<Eigen::Vector3i> blocks;
                for (int i = 0; i < 8; i++) {
                    Eigen::Vector3i block = fragment - variations.col(i);
                    if (block.minCoeff() < 0 || (block - fragmentation + Eigen::Vector3i::Ones()).maxCoeff() >= 0)
                        continue;
                    blocks.push_back(block);
                }

                std::vector<int> submapIndices;
                Eigen::Vector3i submapSize =
                        Eigen::Vector3i::Zero().cwiseMax(fragmentation - Eigen::Vector3i::Ones());
                for (auto block: blocks) {
                    int submapIndex = block.z() * (submapSize.x() * submapSize.y());
                    submapIndex += ((submapSize.y() - 1) * (block.z() % 2) +
                                    block.y() * (int) pow(-1, block.z() % 2)) * submapSize.x();
                    submapIndex += (submapSize.x() - 1) * ((block.y() % 2) ^ (block.z() % 2)) +
                                   block.x() * (int) pow(-1, (block.y() % 2) ^ (block.z() % 2));

                    submapIndices.push_back(submapIndex);
                }
                return submapIndices;
            }

            /**
             * Creates a posegraph object and writes it out as protocol buffer.
             *
             * The method creates a posegraph-object and fills it with the submap. It then chooses fitting parameters
             * or dummy values to convert the posegraph to a file ending with .pbstream.
             *
             * @param submap pointer to a submap3d-object containing the created grid.
             * @param filename name of the file to be created.
             */
            static void
            writeSubmapsOut(std::vector<cartographer::mapping::Submap3D> &submaps, const std::string &filename) {

                // --- Build a pose graph for the submap (use the values from map_builder.lua!) ---
                const std::string code_pbstream = R"text(
                    include "map_builder.lua"
                    MAP_BUILDER.use_trajectory_builder_3d = true
                    return MAP_BUILDER)text";

                // Copied from map_builder_test.cc in SetUp()
                auto file_resolver_pbstream =
                        absl::make_unique<::cartographer::common::ConfigurationFileResolver>(
                                std::vector<std::string>{
                                        std::string(::cartographer::common::kSourceDirectory) +
                                        "/configuration_files"});
                std::unique_ptr<common::LuaParameterDictionary> dict_pbstream =
                        std::make_unique<common::LuaParameterDictionary>(code_pbstream,
                                                                         std::move(file_resolver_pbstream));

                proto::MapBuilderOptions map_builder_options_ = CreateMapBuilderOptions(dict_pbstream.get());
                cartographer::common::ThreadPool my_thread_pool(map_builder_options_.num_background_threads());

                // Copied from map_builder.cc, line 106
                cartographer::mapping::PoseGraph3D posegraph(
                        map_builder_options_.pose_graph_options(),
                        absl::make_unique<optimization::OptimizationProblem3D>(
                                map_builder_options_.pose_graph_options().optimization_problem_options()),
                        &my_thread_pool
                );


                // --- Put the submaps in the pose graph ---

                for (int s = 0; s < (int) submaps.size(); s++) {
                    submaps[s].Finish();

                    proto::Submap proto = submaps[s].ToProto(true);
                    for (int i = 0; i < submaps[s].rotational_scan_matcher_histogram().size(); i++) {
                        proto.mutable_submap_3d()->add_rotational_scan_matcher_histogram(
                                submaps[s].rotational_scan_matcher_histogram()[i]);
                    }
                    proto.mutable_submap_id()->set_submap_index(s);

//                    std::cout << "Size of histogram before proto: " << submap.rotational_scan_matcher_histogram().size()
//                              << std::endl;
//                    std::cout << "Size of histogram after proto: "
//                              << proto.submap_3d().rotational_scan_matcher_histogram().size() << std::endl;


                    posegraph.AddSubmapFromProto(cartographer::transform::Rigid3d::Identity(), proto);

//                    const cartographer::mapping::Submap3D *new_submap;
//                    new_submap = dynamic_cast<const Submap3D *>(posegraph.GetAllSubmapData().begin()->data.submap.get());
//                    std::cout << "Size of histogram in posegraph: "
//                          << new_submap->rotational_scan_matcher_histogram().size() << std::endl;
                }

                // --- Write the pose graph as protoBuffer ---
                proto::TrajectoryBuilderOptionsWithSensorIds my_traj_builder_options;
                std::vector<proto::TrajectoryBuilderOptionsWithSensorIds> trajectory_builder_options;
                trajectory_builder_options.push_back(my_traj_builder_options);

                cartographer::io::ProtoStreamWriter writer(filename);
                cartographer::io::WritePbStream(posegraph, trajectory_builder_options, &writer, true);
                writer.Close();

//                io::ProtoStreamReader stream(
//                        path_to_home + "/hector/src/cartographer/cartographer/io/pointcloud_conversion/" +
//                        "ProtoBuffers/OccupancyProtoBufferTest.pbstream");
//                io::ProtoStreamDeserializer deserializer(&stream);
//                io::SerializedData serialized_data;
//                while (deserializer.ReadNextSerializedData(&serialized_data)) {
//                    std::cout << "Added submap from proto with rot_histo-size "
//                              << serialized_data.submap().submap_3d().rotational_scan_matcher_histogram().size()
//                              << std::endl;
//                }
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

//                cartographer::mapping::buildTestWorldPointCloud(path_to_home);
//                return;

                cartographer::mapping::TSDFDrawer myTSDFDrawer;

                std::unique_ptr<HybridGridTSDF> myHighResTSDF;
                std::vector<std::unique_ptr<HybridGridTSDF>> myHighResTSDFSubgrids;
                std::unique_ptr<HybridGridTSDF> myLowResTSDF;
                std::vector<std::unique_ptr<HybridGridTSDF>> myLowResTSDFSubgrids;

                std::unique_ptr<HybridGrid> myHighResHybridGrid;
                std::vector<std::unique_ptr<HybridGrid>> myHighResHybridGridSubgrids;
                std::unique_ptr<HybridGrid> myLowResHybridGrid;
                std::vector<std::unique_ptr<HybridGrid>> myLowResHybridGridSubgrids;

                std::shared_ptr<open3d::geometry::VoxelGrid> grid_highRes;
                std::shared_ptr<open3d::geometry::VoxelGrid> grid_lowRes;

                std::shared_ptr<open3d::geometry::PointCloud> myPointCloudPointer =
                        std::make_shared<open3d::geometry::PointCloud>();

                // Read and show the input from the .ply-file.
                std::string point_cloud_filename =
                        path_to_home + luaParameterDictionary->GetString("pointcloudPath");
                open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});
                std::cout << "Loaded point cloud with exactly " << myPointCloudPointer->points_.size() << " points."
                          << std::endl;

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

                if (luaParameterDictionary->GetBool("generateTSDF")) {
                    myPointCloudPointer->EstimateNormals();
                    std::cout << "Estimated all normals for the point cloud." << std::endl;

                    myPointCloudPointer->OrientNormalsConsistentTangentPlane(
                            luaParameterDictionary->GetInt("normalOrientationNearestNeighbours"));
                    std::cout << "Oriented all normals by using the tangent plane." << std::endl;
                }


                // Draw all points of the filtered point cloud + their normals
//                open3d::visualization::DrawGeometries({myPointCloudPointer});


// #################################################################################################################
                // Show the VoxelGrid of the loaded point cloud

                // Apply colors to the points
                Eigen::Vector3d maxValues = myPointCloudPointer->GetMaxBound();
                Eigen::Vector3d minValues = myPointCloudPointer->GetMinBound();
                Eigen::Vector3d ranges = maxValues - minValues;
                if (!myPointCloudPointer->HasColors()) {
                    for (const Eigen::Vector3d &p: myPointCloudPointer->points_) {
                        myPointCloudPointer->colors_.emplace_back(
                                Eigen::Vector3d{(p.x() - minValues.x()) * 1.0 / ranges.x(), 0.0, 0.0});
                    }
                }

                float gridVoxelSideLength_highRes = (float) luaParameterDictionary->GetDouble(
                        "absoluteHighResVoxelSize");
                float gridVoxelSideLength_lowRes = (float) luaParameterDictionary->GetDouble("absoluteLowResVoxelSize");

                int numberOfVoxels_highRes = (int) (ranges.x() * ranges.y() * ranges.z() /
                                                    pow(gridVoxelSideLength_highRes, 3));
                std::cout << "Created HighRes-VoxelGrid with " << numberOfVoxels_highRes << " possible voxels."
                          << std::endl;
                std::shared_ptr<open3d::geometry::VoxelGrid> pclVoxelGridPointer =
                        open3d::geometry::VoxelGrid::CreateFromPointCloud(*myPointCloudPointer,
                                                                          gridVoxelSideLength_highRes);

//                myTSDFDrawer.drawTSDF(pclVoxelGridPointer);




// #################################################################################################################
                // Decide on a fragmentation of the whole map

                cartographer::mapping::ValueConversionTables my_vct;

                // High Resolution
                Eigen::Vector3i minIndexHybridGrid_highRes =
                        (myPointCloudPointer->GetMinBound() / gridVoxelSideLength_highRes).array().ceil().cast<int>();
                Eigen::Vector3i maxIndexHybridGrid_highRes =
                        (myPointCloudPointer->GetMaxBound() / gridVoxelSideLength_highRes).array().ceil().cast<int>();
                Eigen::Vector3i gridSizeHybridGrid_highRes = maxIndexHybridGrid_highRes - minIndexHybridGrid_highRes;

                // Low Resolution
                Eigen::Vector3i minIndexHybridGrid_lowRes =
                        (myPointCloudPointer->GetMinBound() /
                         gridVoxelSideLength_lowRes).array().ceil().cast<int>();
                Eigen::Vector3i maxIndexHybridGrid_lowRes =
                        (myPointCloudPointer->GetMaxBound() /
                         gridVoxelSideLength_lowRes).array().ceil().cast<int>();
                Eigen::Vector3i gridSizeHybridGrid_lowRes = maxIndexHybridGrid_lowRes - minIndexHybridGrid_lowRes;

                int numberOfSubmaps = luaParameterDictionary->GetInt("numberOfSubmaps");

                // Fragmentation is the same for low and high resolution
                Eigen::Vector3i fragmentation = getFragmentationForHybridGrid(numberOfSubmaps,
                                                                              gridSizeHybridGrid_highRes);
                numberOfSubmaps = (fragmentation - Eigen::Vector3i::Ones()).prod();

                Eigen::Matrix<int, 3, 8> variations;
                variations << Eigen::Matrix<int, 3, 4>::Identity(), Eigen::Matrix<int, 3, 4>::Ones() -
                                                                    Eigen::Matrix<int, 3, 4>::Identity();

// #################################################################################################################
                // Build a HybridGridTSDF = cartographer's representation of a TSDF

                if (luaParameterDictionary->GetBool("generateTSDF")) {

                    float relativeHighResTruncationDistance =
                            (float) luaParameterDictionary->GetDouble("relativeHighResTruncationDistance");
                    float relativeLowResTruncationDistance =
                            (float) luaParameterDictionary->GetDouble("relativeLowResTruncationDistance");
                    float maxWeight = 100.0;
                    float absoluteHighResTruncationDistance =
                            relativeHighResTruncationDistance * gridVoxelSideLength_highRes;
                    float absoluteLowResTruncationDistance =
                            relativeLowResTruncationDistance * gridVoxelSideLength_lowRes;

                    for (int i = 0; i < numberOfSubmaps; i++) {
                        myHighResTSDFSubgrids.push_back(absl::make_unique<HybridGridTSDF>(
                                gridVoxelSideLength_highRes, relativeHighResTruncationDistance, maxWeight, &my_vct));
                        myLowResTSDFSubgrids.push_back(absl::make_unique<HybridGridTSDF>(
                                gridVoxelSideLength_lowRes, relativeLowResTruncationDistance, maxWeight, &my_vct));
                    }

                    myHighResTSDF = absl::make_unique<HybridGridTSDF>(
                            gridVoxelSideLength_highRes, relativeHighResTruncationDistance, maxWeight, &my_vct);

                    myLowResTSDF = absl::make_unique<HybridGridTSDF>(
                            gridVoxelSideLength_lowRes, relativeLowResTruncationDistance, maxWeight, &my_vct);

                    // Build the TSDF by raytracing every point/normal pair from the point cloud
                    for (long unsigned int i = 0; i < myPointCloudPointer->points_.size(); i++) {
                        raycastPointWithNormal(
                                myPointCloudPointer->points_.at(i).cast<float>(),
                                myPointCloudPointer->normals_.at(i).cast<float>(),
                                absoluteHighResTruncationDistance,
                                myHighResTSDF.get());

                        raycastPointWithNormal(
                                myPointCloudPointer->points_.at(i).cast<float>(),
                                myPointCloudPointer->normals_.at(i).cast<float>(),
                                absoluteLowResTruncationDistance,
                                myLowResTSDF.get());
                    }

                    for (auto nextVoxel: *myHighResTSDF) {
                        std::vector<int> submapIndices =
                                getSubmapIndicesForVoxel(nextVoxel.first - minIndexHybridGrid_highRes.array(),
                                                         gridSizeHybridGrid_highRes, fragmentation, variations);
                        for (int index: submapIndices) {
                            myHighResTSDFSubgrids.at(index)->SetCell(nextVoxel.first,
                                                                     myHighResTSDF->GetTSD(nextVoxel.first),
                                                                     myHighResTSDF->GetWeight(nextVoxel.first));
                        }
                    }
                    for (auto nextVoxel: *myLowResTSDF) {
                        std::vector<int> submapIndices =
                                getSubmapIndicesForVoxel(nextVoxel.first - minIndexHybridGrid_lowRes.array(),
                                                         gridSizeHybridGrid_lowRes, fragmentation, variations);
                        for (int index: submapIndices) {
                            myLowResTSDFSubgrids.at(index)->SetCell(nextVoxel.first,
                                                                    myLowResTSDF->GetTSD(nextVoxel.first),
                                                                    myLowResTSDF->GetWeight(nextVoxel.first));
                        }
                    }

//                    for (int i = 0; i < numberOfSubmaps; i++) {
//                        std::cout << "Subgrid " << i << std::endl;
//                        std::shared_ptr<open3d::geometry::VoxelGrid> subgrid = convertTSDFToVoxelGrid(
//                                myHighResTSDFSubgrids.at(i).get(),
//                                gridVoxelSideLength_highRes, absoluteHighResTruncationDistance);
//                        myTSDFDrawer.drawTSDF(subgrid);
//                    }

                    grid_highRes = convertTSDFToVoxelGrid(
                            myHighResTSDF.get(), gridVoxelSideLength_highRes, absoluteHighResTruncationDistance);

                    grid_lowRes = convertTSDFToVoxelGrid(
                            myLowResTSDF.get(), gridVoxelSideLength_lowRes, absoluteLowResTruncationDistance);
                }


// #################################################################################################################
                    // Build a HybridGrid = cartographer's representation of an occupancy grid
                else {

                    for (int i = 0; i < numberOfSubmaps; i++) {
                        myHighResHybridGridSubgrids.push_back(absl::make_unique<HybridGrid>(
                                gridVoxelSideLength_highRes, &my_vct));
                        myLowResHybridGridSubgrids.push_back(absl::make_unique<HybridGrid>(
                                gridVoxelSideLength_lowRes, &my_vct));
                    }

                    myHighResHybridGrid = absl::make_unique<HybridGrid>(
                            gridVoxelSideLength_highRes, &my_vct);
                    myLowResHybridGrid = absl::make_unique<HybridGrid>(
                            gridVoxelSideLength_lowRes, &my_vct);

                    for (const auto &nextPoint: myPointCloudPointer->points_) {
                        Eigen::Array3i update_cell_index = myHighResHybridGrid->GetCellIndex(nextPoint.cast<float>());
                        myHighResHybridGrid->SetProbability(update_cell_index, 1.0);

                        update_cell_index = myLowResHybridGrid->GetCellIndex(nextPoint.cast<float>());
                        myLowResHybridGrid->SetProbability(update_cell_index, 1.0);
                    }

                    for (auto nextVoxel: *myHighResHybridGrid) {
                        std::vector<int> submapIndices =
                                getSubmapIndicesForVoxel(nextVoxel.first - minIndexHybridGrid_highRes.array(),
                                                         gridSizeHybridGrid_highRes, fragmentation, variations);
                        for (int index: submapIndices) {
                            myHighResHybridGridSubgrids.at(index)->SetProbability(nextVoxel.first, 1.0);
                        }
                    }
                    for (auto nextVoxel: *myLowResHybridGrid) {
                        std::vector<int> submapIndices =
                                getSubmapIndicesForVoxel(nextVoxel.first - minIndexHybridGrid_lowRes.array(),
                                                         gridSizeHybridGrid_lowRes, fragmentation, variations);
                        for (int index: submapIndices) {
                            myLowResHybridGridSubgrids.at(index)->SetProbability(nextVoxel.first, 1.0);
                        }
                    }

//                    for (int i = 0; i < numberOfSubmaps; i++) {
//                        std::cout << "Subgrid " << i << std::endl;
//                        std::shared_ptr<open3d::geometry::VoxelGrid> subgrid = convertOccupancyGridToVoxelGrid(
//                                dynamic_cast<HybridGrid *>(myHighResHybridGridSubgrids.at(i).get()),
//                                gridVoxelSideLength_highRes);
//                        myTSDFDrawer.drawTSDF(subgrid);
//                    }

                    grid_highRes = convertOccupancyGridToVoxelGrid(
                            dynamic_cast<HybridGrid *>(myHighResHybridGrid.get()), gridVoxelSideLength_highRes);
                    grid_lowRes = convertOccupancyGridToVoxelGrid(
                            dynamic_cast<HybridGrid *>(myLowResHybridGrid.get()), gridVoxelSideLength_lowRes);
                }


// #################################################################################################################
                // Draw and show the images of the TSDF or Occupancy Grid
                // std::cout << "Press the left/right keys to slice through the voxel grid!" << std::endl;
                // std::cout << "Press the key >o< to change the slicing orientation." << std::endl;
//                myTSDFDrawer.drawTSDF(grid_highRes);
//                myTSDFDrawer.drawTSDF(grid_lowRes);

// #################################################################################################################
                // Save some slices as png (uses the high resolution grid!)

                if (luaParameterDictionary->GetBool("saveSlicesAsPNG")) {
                    std::string imgfilename;
                    imgfilename = path_to_home +
                                  "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                                  + "slice_img_x.png";
                    myTSDFDrawer.saveSliceAsPNG(0, 0, imgfilename.c_str(), grid_highRes);

                    imgfilename = path_to_home +
                                  "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                                  + "slice_img_y.png";
                    myTSDFDrawer.saveSliceAsPNG(0, 1, imgfilename.c_str(), grid_highRes);

                    for (int i = 0; i < 6; i++) {
                        imgfilename = path_to_home +
                                      "/hector/src/cartographer/cartographer/io/pointcloud_conversion/images/"
                                      + "slice_img_z" + std::to_string(i) + ".png";

                        myTSDFDrawer.saveSliceAsPNG(luaParameterDictionary->GetInt("imageSliceIndex") + 3 * i, 2,
                                                    imgfilename.c_str(),
                                                    grid_highRes);
                    }
                }


// #################################################################################################################
                // Build a ProtoBuffer

                int histogram_size = 120;

                std::vector<sensor::PointCloud> sub_pointclouds;
                for (int i = 0; i < numberOfSubmaps; i++) {
                    sub_pointclouds.emplace_back();
                }

                for (const auto &point: myPointCloudPointer->points_) {
                    Eigen::Vector3i gridVoxel;
                    std::vector<int> submapIndices;
                    if (luaParameterDictionary->GetBool("generateTSDF")) {
                        gridVoxel =
                                myHighResTSDF->GetCellIndex(point.cast<float>()) - minIndexHybridGrid_highRes.array();
                        submapIndices = getSubmapIndicesForVoxel(gridVoxel, gridSizeHybridGrid_highRes, fragmentation,
                                                                 variations);
                    } else {
                        gridVoxel = myHighResHybridGrid->GetCellIndex(point.cast<float>()) -
                                    minIndexHybridGrid_highRes.array();
                        submapIndices = getSubmapIndicesForVoxel(gridVoxel, gridSizeHybridGrid_highRes, fragmentation,
                                                                 variations);
                    }

                    for (int i: submapIndices) {
                        sub_pointclouds.at(i).push_back(
                                {Eigen::Vector3f((float) point.x(), (float) point.y(), (float) point.z())});
                    }
                }

                std::vector<Eigen::VectorXf> histograms;
                for (const auto &subcloud: sub_pointclouds) {
                    histograms.push_back(
                            scan_matching::RotationalScanMatcher::ComputeHistogram(subcloud, histogram_size));
                }

//                for (const auto &histo: histograms) {
//                    for (int i = 0; i < histo.size(); i++) {
//                        std::cout << histo[i] << ", ";
//                    }
//                    std::cout << std::endl;
//                }

                const cartographer::common::Time my_time = cartographer::common::FromUniversal(100);
                std::vector<cartographer::mapping::Submap3D> submaps;

                if (luaParameterDictionary->GetBool("generateTSDF")) {
                    for (int i = 0; i < (int) histograms.size(); i++) {
                        submaps.emplace_back(
                                cartographer::transform::Rigid3d::Identity(),
                                std::move(myLowResTSDFSubgrids.at(i)),
                                std::move(myHighResTSDFSubgrids.at(i)),
                                histograms.at(i), &my_vct, my_time);
                    }
                    writeSubmapsOut(submaps, path_to_home +
                                             "/hector/src/cartographer/cartographer/io/pointcloud_conversion/ProtoBuffers/" +
                                             luaParameterDictionary->GetString("outputName") + ".pbstream");
                } else {
                    for (int i = 0; i < (int) histograms.size(); i++) {
                        submaps.emplace_back(
                                cartographer::transform::Rigid3d::Identity(),
                                std::move(myLowResHybridGridSubgrids.at(i)),
                                std::move(myHighResHybridGridSubgrids.at(i)),
                                histograms.at(i), &my_vct, my_time);
                    }
                    writeSubmapsOut(submaps, path_to_home +
                                             "/hector/src/cartographer/cartographer/io/pointcloud_conversion/ProtoBuffers/" +
                                             luaParameterDictionary->GetString("outputName") + ".pbstream");
                }
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
