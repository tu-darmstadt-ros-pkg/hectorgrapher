#ifndef CARTOGRAPHER_POINTCLOUD_CREATOR_H
#define CARTOGRAPHER_POINTCLOUD_CREATOR_H

#include <random>
#include "Eigen/Core"

#ifdef WITH_OPEN3D
#include "open3d/Open3D.h"
#endif

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        void buildTestWorldPointCloud(const std::string& path_to_home);

    }
}
#endif

#endif //CARTOGRAPHER_POINTCLOUD_CREATOR_H
