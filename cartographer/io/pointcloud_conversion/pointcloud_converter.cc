#include <cmath>
#include <fstream>
#include <string>

#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

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