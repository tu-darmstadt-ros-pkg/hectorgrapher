#include <cmath>
#include <fstream>
#include <string>

//#include "cartographer/transform/transform.h"
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

//#include "cartographer/common/lua_parameter_dictionary.h"
//#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/configuration_file_resolver.h"


#ifdef WITH_OPEN3D

#include "open3d/Open3D.h"

#endif

DEFINE_string(config_file, "", "LUA-file containing the configuration options");

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFBuilder {

            std::shared_ptr<open3d::geometry::VoxelGrid> tsdfPointer;
            int sliceIndex;
            int sliceOrientation;

            cartographer::common::LuaParameterDictionary* luaParameterDictionary;

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

            /**
             * Show every voxel of a voxel grid with a z-index one higher than already seen.
             *
             * This method is a callback method for "drawTSDF".
             *
             * @param visualizer control class for the open3d window.
             * @return true.
             */
            bool loopForwardThroughSlices(open3d::visualization::Visualizer *visualizer) {
                return loopThroughSlices(visualizer, -1);
            }

            /**
             * Show every voxel of a voxel grid with a z-index one lower than already seen.
             *
             * This method is a callback method for "drawTSDF".
             *
             * @param visualizer control class for the open3d window.
             * @return true.
             */
            bool loopBackwardThroughSlices(open3d::visualization::Visualizer *visualizer) {
                return loopThroughSlices(visualizer, 1);
            }

            /**
             * Show every voxel of a voxel grid with a certain z-index. As a result, produce a "slice" of a grid.
             *
             * By copying the voxel size and origin point of the original voxel grid, the slice doesn't change position.
             * Since every call of this method includes an iteration of all voxels, this method could take a lot of
             * time when confronted with large areas or small voxels.
             *
             * @param visualizer control class for the open3d window.
             * @param velocity controls how the position of the slice is changed. Useful numbers are "1" or "-1".
             * @return true in any case.
             */
            bool loopThroughSlices(open3d::visualization::Visualizer *visualizer, int velocity) {
                sliceIndex += velocity;
//                std::cout << "Show slice " << sliceIndex << std::endl;

                std::shared_ptr<open3d::geometry::VoxelGrid> slicedVoxelGridPointer =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());

                slicedVoxelGridPointer->voxel_size_ = tsdfPointer->voxel_size_;
                slicedVoxelGridPointer->origin_ = tsdfPointer->origin_;

                for (open3d::geometry::Voxel nextVoxel : tsdfPointer->GetVoxels()) {
                    if (nextVoxel.grid_index_(sliceOrientation) == sliceIndex) {
                        slicedVoxelGridPointer->AddVoxel(nextVoxel);
                    }
                }
                visualizer->ClearGeometries();
                visualizer->AddGeometry(slicedVoxelGridPointer, false);
                return true;
            }

            /**
             * Change the dimension of the displayed slices of a VoxelGrid.
             *
             * This method is a callback method for "drawTSDF".
             *
             * @param visualizer control class for the open3d window.
             * @return true in any case.
             */
            bool changeOrientation(open3d::visualization::Visualizer *visualizer) {
                sliceOrientation = (sliceOrientation + 1) % 3;
                sliceIndex = 1;
                drawFullView(visualizer);
                return true;
            }

            /**
             * Show a whole voxel grid.
             *
             * This method is a callback method for "drawTSDF".
             *
             * @param visualizer control class for the open3d window.
             * @return true in any case.
             */
            bool drawFullView(open3d::visualization::Visualizer *visualizer) {
                visualizer->ClearGeometries();
                visualizer->AddGeometry(tsdfPointer, false);
                return true;
            }

            /**
             * Draw a whole voxel grid in an open3d window and control the display of slices of the grid.
             *
             * The left and right arrow move the slice forward and backward.
             * The down arrow resets the view to the whole grid.
             *
             * @param voxelGridPointer pointer to an open3d's representation of a voxel grid to be displayed.
             */
            void drawTSDF(const std::shared_ptr<open3d::geometry::VoxelGrid> &voxelGridPointer) {
                tsdfPointer = voxelGridPointer;

                std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> myMap;

                std::function<bool(open3d::visualization::Visualizer *)> forwardSlicing =
                        std::bind(&TSDFBuilder::loopForwardThroughSlices, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_LEFT, forwardSlicing));

                std::function<bool(open3d::visualization::Visualizer *)> backwardSlicing =
                        std::bind(&TSDFBuilder::loopBackwardThroughSlices, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_RIGHT, backwardSlicing));

                std::function<bool(open3d::visualization::Visualizer *)> orientationChanging =
                        std::bind(&TSDFBuilder::changeOrientation, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_O, orientationChanging));

                std::function<bool(open3d::visualization::Visualizer *)> fullView =
                        std::bind(&TSDFBuilder::drawFullView, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_F, fullView));

                open3d::visualization::DrawGeometriesWithKeyCallbacks({tsdfPointer}, myMap);
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
                luaParameterDictionary = new cartographer::common::LuaParameterDictionary(code, std::move(file_resolver));
//                std::cout << luaParameterDictionary->GetInt("testvalue") << std::endl;

                sliceIndex = 1;
                sliceOrientation = 2;       // The z-dimension is the default slice direction
            }


            void run() {

                std::shared_ptr<open3d::geometry::PointCloud> myPointCloudPointer =
                        std::make_shared<open3d::geometry::PointCloud>();

                // Generate a cloud in shape of a cube. Don't show the input from the .ply-file.
                if(luaParameterDictionary->GetBool("generateCubicPointcloud")) {
                    myPointCloudPointer = generateCubicPointCloud(
                            luaParameterDictionary->GetDouble("sidelengthCubicPointcloud"),
                            luaParameterDictionary->GetDouble("distancePointsCubicPointcloud"),
                            luaParameterDictionary->GetDouble("noiseCubicPointcloud")   );
                }
                else {
                    // Read and show the input from the .ply-file.
                    std::string point_cloud_filename = luaParameterDictionary->GetString("pointcloudPath");
                    open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});
                    std::cout << "Loaded point cloud with " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if(luaParameterDictionary->GetBool("uniformDownSample")) {
                    int sampleRate = luaParameterDictionary->GetInt("sampleRateUniformDownSample");
                    myPointCloudPointer = myPointCloudPointer->UniformDownSample(sampleRate);
                    std::cout << "Downsampled to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if(luaParameterDictionary->GetBool("removeRadiusOutliers")) {
                    myPointCloudPointer = std::get<0>(myPointCloudPointer->RemoveRadiusOutliers(
                            luaParameterDictionary->GetInt("neighborsInSphereRadiusOutlier"),
                            luaParameterDictionary->GetDouble("neighborsInSphereRadiusOutlier") ));
                    std::cout << "Removed outliers to " << myPointCloudPointer->points_.size() << " points."
                              << std::endl;
                }

                if(luaParameterDictionary->GetBool("cutRoofZAxis")) {
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
                        luaParameterDictionary->GetInt("normalOrientationNearestNeighbours")    );
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

//                float gridVoxelSideLength = 0.002;
//                float gridVoxelSideLength = 0.2;
                float gridVoxelSideLength = luaParameterDictionary->GetDouble("absoluteVoxelSize");
                int numberOfVoxels = (int) (ranges.x() * ranges.y() * ranges.z() / pow(gridVoxelSideLength, 3));
                std::cout << "Created VoxelGrid with " << numberOfVoxels << " possible voxels." << std::endl;

                std::shared_ptr<open3d::geometry::VoxelGrid> pclVoxelGridPointer =
                        open3d::geometry::VoxelGrid::CreateFromPointCloud(*myPointCloudPointer, gridVoxelSideLength);

                drawTSDF(pclVoxelGridPointer);



// #################################################################################################################
                // Build a HybridGridTSDF = cartographer's representation of a TSDF


//                float absoluteTruncationDistance = 0.025;
//                float absoluteTruncationDistance = 4.0;
                float absoluteTruncationDistance = luaParameterDictionary->GetDouble("absoluteTruncationDistance");
                float maxWeight = luaParameterDictionary->GetDouble("maxTSDFWeight");  // Todo: Was sollte das maximale Gewicht sein?
                float relativeTruncationDistance = absoluteTruncationDistance / gridVoxelSideLength;

                cartographer::mapping::ValueConversionTables myValueConversionTable;
                cartographer::mapping::HybridGridTSDF myHybridGridTSDF(
                        gridVoxelSideLength, relativeTruncationDistance, maxWeight, &myValueConversionTable);

//                for(int i=0; i<myHybridGrid.grid_size(); i++) {
//                    for(int j=0; j<myHybridGrid.grid_size(); j++) {
//                        for(int k=0; k<myHybridGrid.grid_size(); k++) {
//                            myHybridGrid.SetCell({i,j,k}, 3.0, 0.0);
//                        }
//                    }
//                }


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
                drawTSDF(tsdfVoxelGridPointer);

//                myHybridGridTSDF.ToProto();

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



// Possible Program Arguments
// -pointcloud_file "/home/leo/Downloads/Halle-DRZ-Modell-innen-Flug1-2020-12-09.ply"
