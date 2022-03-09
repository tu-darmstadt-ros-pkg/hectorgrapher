#include "cartographer/io/pointcloud_conversion/pointcloud_creator.h"

namespace cartographer {
    namespace mapping {

/**
 * Builds a predefined world to test the generation of the grids in a simple environment.
 *
 * The method builds a world consisting of two blocks, samples points from them and writes
 * the point cloud in a file in the Downloads-folder.
 * This method should be used entirely independent from the rest of the program.
 */
        void buildTestWorldPointCloud(const std::string &path_to_home) {
            double resolution = 0.1;
            float std_dev = 0.01;

            std::vector<Eigen::Vector3d> pos = {{0.0,  0.0,   1.5},
                                                {2.0,  3.0,   0.5},
                                                {2.0,  23.0,  0.5},
                                                {-1.5, 10.0,  1.0},
                                                {-2.0, -7.0,  0.5},
                                                {2.0,  -23.0, 0.5}};
            std::vector<Eigen::Vector3d> size = {{5.0, 47.0, 3.0},
                                                 {1.0, 1.0,  1.0},
                                                 {1.0, 1.0,  1.0},
                                                 {2.0, 2.0,  2.0},
                                                 {1.0, 1.0,  1.0},
                                                 {1.0, 1.0,  1.0}};
            std::vector<bool> create_roof = {false, true, true, true, true, true};

            std::vector<Eigen::Vector3d> listOfPoints;

            std::vector<Eigen::Vector3d> mins;
            std::vector<Eigen::Vector3d> maxs;

            for (int i = 0; i < (int) pos.size(); i++) {
                mins.emplace_back(Eigen::Vector3d(-size[i] / 2.0 + pos[i]));
                maxs.emplace_back(Eigen::Vector3d(size[i] / 2.0 + pos[i]));
            }

            for (int i = 0; i < (int) pos.size(); i++) {
                for (int ix = 0; ix < size[i].x() / resolution + 1e-5; ix++) {
                    for (int iy = 0; iy < size[i].y() / resolution + 1e-5; iy++) {
                        listOfPoints.emplace_back(
                                Eigen::Vector3d(mins[i].x() + resolution * ix, mins[i].y() + iy * resolution,
                                                mins[i].z()));
                        if (create_roof[i])
                            listOfPoints.emplace_back(
                                    Eigen::Vector3d(mins[i].x() + ix * resolution, mins[i].y() + iy * resolution,
                                                    maxs[i].z()));
                    }
                }

                for (int ix = 0; ix < size[i].x() / resolution + 1e-5; ix++) {
                    for (int iz = 0; iz < size[i].z() / resolution + 1e-5; iz++) {
                        if (i != 0)
                            listOfPoints.emplace_back(Eigen::Vector3d(mins[i].x() + resolution * ix, mins[i].y(),
                                                                      mins[i].z() + iz * resolution));
                        listOfPoints.emplace_back(Eigen::Vector3d(mins[i].x() + resolution * ix, maxs[i].y(),
                                                                  mins[i].z() + iz * resolution));
                    }
                }

                for (int iz = 0; iz < size[i].z() / resolution + 1e-5; iz++) {
                    for (int iy = 0; iy < size[i].y() / resolution + 1e-5; iy++) {
                        listOfPoints.emplace_back(Eigen::Vector3d(mins[i].x(), mins[i].y() + iy * resolution,
                                                                  mins[i].z() + iz * resolution));
                        listOfPoints.emplace_back(Eigen::Vector3d(maxs[i].x(), mins[i].y() + iy * resolution,
                                                                  mins[i].z() + iz * resolution));
                    }
                }
            }

            std::random_device rd;
            std::default_random_engine rng(rd());
            std::shuffle(listOfPoints.begin(), listOfPoints.end(), rng);

            std::normal_distribution<float> normal_distribution(0, std_dev);
            for (auto &listOfPoint: listOfPoints) {
                listOfPoint.x() += normal_distribution(rng);
                listOfPoint.y() += normal_distribution(rng);
                listOfPoint.z() += normal_distribution(rng);
            }

            open3d::geometry::PointCloud myPointCloud(listOfPoints);
            open3d::io::WritePointCloudOption options;
            open3d::io::WritePointCloudToPLY(path_to_home + "/Downloads/xxx.ply", myPointCloud, options);
        }
    }
}