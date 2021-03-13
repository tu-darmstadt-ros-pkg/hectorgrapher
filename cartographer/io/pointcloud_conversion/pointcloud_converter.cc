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

#ifdef WITH_OPEN3D

#include "open3d/Open3D.h"

#endif

DEFINE_string(pointcloud_file, "", "File containing the point-cloud input.");

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFBuilder {

            std::shared_ptr<open3d::geometry::VoxelGrid> tsdfPointer;
            int sliceIndex;

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
            std::shared_ptr<open3d::geometry::PointCloud> generateCubicPointCloud() {
                // Initialize an empty Point Cloud in the data format of cartographer
                cartographer::sensor::PointCloud cartographer_cloud;

                // Fill the Point Cloud with points (shape of a cube)
                float distanceBetweenPoints = 0.004;
                float cubeSideLength = 0.1;
                float noise = 0.0;
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
                    if (nextVoxel.grid_index_.z() == sliceIndex) {
                        slicedVoxelGridPointer->AddVoxel(nextVoxel);
                    }
                }
                visualizer->ClearGeometries();
                visualizer->AddGeometry(slicedVoxelGridPointer, false);
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
                myMap.insert(std::make_pair((int) GLFW_KEY_RIGHT, forwardSlicing));

                std::function<bool(open3d::visualization::Visualizer *)> backwardSlicing =
                        std::bind(&TSDFBuilder::loopBackwardThroughSlices, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_LEFT, backwardSlicing));

                std::function<bool(open3d::visualization::Visualizer *)> fullView =
                        std::bind(&TSDFBuilder::drawFullView, this, std::placeholders::_1);
                myMap.insert(std::make_pair((int) GLFW_KEY_DOWN, fullView));

                open3d::visualization::DrawGeometriesWithKeyCallbacks({tsdfPointer}, myMap);
            }


            // Abgeschrieben von cartographer::mapping:3d::tsdf_range_data_inserter_3d
            // Sample TSDF Generation in Cartographer --> see evaluation/scan_matching_evaluation.cc
            // Todo: Macht bis jetzt was sehr komisches... ;)

            void raycastPointWithNormal(const Eigen::Vector3f &hit,
                                        const Eigen::Vector3f &normal,
                                        const float truncation_distance,
                                        HybridGridTSDF *tsdf) const {
                const Eigen::Vector3f ray_begin = hit - truncation_distance * normal;
                const Eigen::Vector3f ray_end = hit + truncation_distance * normal;

                const Eigen::Array3i begin_cell = tsdf->GetCellIndex(ray_begin);
                const Eigen::Array3i end_cell = tsdf->GetCellIndex(ray_end);
                const Eigen::Array3i delta = end_cell - begin_cell;

                const int num_samples = delta.cwiseAbs().maxCoeff();

                for (int position = 0; position <= num_samples; ++position) {
                    const Eigen::Array3i update_cell_index =
                            begin_cell +
                            (delta.cast<float>() * float(position) / float(num_samples)).round().cast<int>();

                    Eigen::Vector3f cell_center = tsdf->GetCenterOfCell(update_cell_index);
                    float update_tsd = (cell_center - hit).norm();
                    if ((cell_center - hit).dot(normal) < 0) {
                        update_tsd = -update_tsd;
                    }

                    // Ignoriert Gewichte, ändert TSD einfach in den niedrigeren Wert.
                    // Warum brauchen wir überhaupt Gewichte?
                    if (!tsdf->IsKnown(update_cell_index) ||
                        std::abs(tsdf->GetTSD(update_cell_index)) > std::abs(update_tsd)) {
                        tsdf->SetCell(update_cell_index, update_tsd, tsdf->GetWeight(update_cell_index));
                    }
                }
            }

            /**
             * Prepare the drawing of cartographer's HybridGridTSDF by converting it in open3d's VoxelGrid.
             *
             * For every voxel in HybridGridTSDF, the method colors a voxel in VoxelGrid blue.
             * // Todo: Color the voxels by looking at the TSD at this voxel.
             *
             * @param hybridGrid pointer to a filled TSDF to be converted.
             * @param voxelSideLength edge length of the voxels of HybridGridTSDF.
             * @return a shared pointer to a VoxelGrid with origin (0,0,0).
             */
            std::shared_ptr<open3d::geometry::VoxelGrid>
            convertHybridGridToVoxelGrid(cartographer::mapping::HybridGridTSDF *hybridGrid, float voxelSideLength) {

                std::shared_ptr<open3d::geometry::VoxelGrid> voxelGrid =
                        std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());
                voxelGrid->voxel_size_ = voxelSideLength;

                for (std::pair<Eigen::Array<int, 3, 1>, TSDFVoxel> nextVoxel : *hybridGrid) {
                    Eigen::Vector3i cellIndex = nextVoxel.first;

                    Eigen::Vector3d color;
                    if (hybridGrid->GetTSD(cellIndex) > 0) {
                        color = {0.0, 0.0, hybridGrid->GetTSD(cellIndex) * 100};
                    } else {
                        color = {hybridGrid->GetTSD(cellIndex) * -100, 0.0, 0.0};
                    }

//                    std::cout << cellIndex.x() << " " << cellIndex.y() << " " << cellIndex.z() << std::endl;
                    open3d::geometry::Voxel newVoxel = open3d::geometry::Voxel(cellIndex, color);
                    voxelGrid->AddVoxel(newVoxel);
                }
                return voxelGrid;
            }

        public:
            TSDFBuilder() {
                sliceIndex = 0;
            }


            void run(const std::string &point_cloud_filename) {

                std::shared_ptr<open3d::geometry::PointCloud> myPointCloudPointer =
                        std::make_shared<open3d::geometry::PointCloud>();

                // Generate a cloud in shape of a cube. Don't show the input from the .ply-file.
//                myPointCloudPointer = generateCubicPointCloud();

                // Read and show the input from the .ply-file.
                open3d::io::ReadPointCloud(point_cloud_filename, *myPointCloudPointer, {"auto", true, true, true});

                myPointCloudPointer->EstimateNormals();
                myPointCloudPointer->OrientNormalsConsistentTangentPlane(9);
                open3d::visualization::DrawGeometries({myPointCloudPointer});


// #################################################################################################################


                // Show the VoxelGrid of the loaded point cloud
                Eigen::Vector3d maxValues = myPointCloudPointer->GetMaxBound();
                Eigen::Vector3d minValues = myPointCloudPointer->GetMinBound();
                Eigen::Vector3d ranges = maxValues - minValues;
                if (!myPointCloudPointer->HasColors()) {
                    for (const Eigen::Vector3d &p : myPointCloudPointer->points_) {
                        myPointCloudPointer->colors_.emplace_back(
                                Eigen::Vector3d{(p.x() - minValues.x()) * 1.0 / ranges.x(), 0.0, 0.0});
                    }
                }
                // Comment the last nine rows to draw a Voxel Grid with no colors.
                float gridVoxelSideLength = 0.002;
                std::shared_ptr<open3d::geometry::VoxelGrid> pclVoxelGridPointer =
                        open3d::geometry::VoxelGrid::CreateFromPointCloud(*myPointCloudPointer, gridVoxelSideLength);

                drawTSDF(pclVoxelGridPointer);


// #################################################################################################################

                // Build a HybridGridTSDF = cartographer's representation of a TSDF
                float tsdfVoxelSideLength = 0.002;
                float absoluteTruncationDistance = 0.01;
                float maxWeight = 10.0;           // Todo: Wird bisher nicht verwendet.
                float relativeTruncationDistance = absoluteTruncationDistance / tsdfVoxelSideLength;

                cartographer::mapping::ValueConversionTables myValueConversionTable;
                cartographer::mapping::HybridGridTSDF myHybridGridTSDF(
                        tsdfVoxelSideLength, relativeTruncationDistance, maxWeight, &myValueConversionTable);

//                for(int i=0; i<myHybridGrid.grid_size(); i++) {
//                    for(int j=0; j<myHybridGrid.grid_size(); j++) {
//                        for(int k=0; k<myHybridGrid.grid_size(); k++) {
//                            myHybridGrid.SetCell({i,j,k}, 3.0, 0.0);
//                        }
//                    }
//                }


// #################################################################################################################

                // Build the TSDF by raytracing every point/normal pair from the point cloud
                for (long unsigned int i = 0; i < myPointCloudPointer->points_.size(); i++) {
                    raycastPointWithNormal(
                            myPointCloudPointer->points_.at(i).cast<float>(),
                            myPointCloudPointer->normals_.at(i).cast<float>(),
                            absoluteTruncationDistance,
                            &myHybridGridTSDF);
                }


// #################################################################################################################

                // Show the VoxelGrid of the TSDF
                std::shared_ptr<open3d::geometry::VoxelGrid> tsdfVoxelGridPointer =
                        convertHybridGridToVoxelGrid(&myHybridGridTSDF, tsdfVoxelSideLength);
                drawTSDF(tsdfVoxelGridPointer);

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