#ifndef CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
#define CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H

#include <cartographer/sensor/point_cloud.h>
#include <fstream>

namespace cartographer {
namespace evaluation {

class ScanCloudGenerator {
 public:
  enum class ModelType { SQUARE, RECTANGLE, CIRCLE, RECTANGLE_RADAR};
  explicit ScanCloudGenerator(cartographer::sensor::PointCloud *scan_cloud);
  explicit ScanCloudGenerator(cartographer::sensor::PointCloud *scan_cloud,
                              float resolution);
  void generateSquare(float size, float noise_std_dev);
  void generateRectangle(float size_x, float size_y, float noise_std_dev);
  void generateRectangleRadar(float size_x, float size_y, float noise_std_dev);
  void generateCircle(float radius, float noise_std_dev);
  void generateOutlier(float size_x, float size_y, float outlier_ratio,
                       float outlier_dist, ModelType model_type);
  void removePoints(float sparse_ratio);
  void generateFromCsv(const std::string &path, int start_scan,
                       int scans_to_load);
  Eigen::Vector3f getStdDev();
  double error_sqr;

 private:
  cartographer::sensor::PointCloud *cloud;
  float resolution_;
//  double std_dev_x;
//  double std_dev_y;
//  double std_dev_xy;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
