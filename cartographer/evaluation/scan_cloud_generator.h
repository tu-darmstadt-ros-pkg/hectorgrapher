#ifndef CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
#define CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H

#include <cartographer/sensor/point_cloud.h>
namespace cartographer {
namespace evaluation {

class ScanCloudGenerator {
 public:
  ScanCloudGenerator();
  ScanCloudGenerator(float resolution);
  void generateSquare(cartographer::sensor::PointCloud& cloud, float size,
                      float noise_std_dev);
  void generateRectangle(cartographer::sensor::PointCloud& cloud, float size_x,
                         float size_y, float noise_std_dev);
  void generateCircle(cartographer::sensor::PointCloud& cloud, float radius,
                      float noise_std_dev);

  enum class ModelType { SQUARE, RECTANGLE, CIRCLE };

 private:
  float resolution_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
