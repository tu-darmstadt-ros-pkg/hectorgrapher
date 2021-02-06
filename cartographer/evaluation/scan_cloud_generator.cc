#include "cartographer/evaluation/scan_cloud_generator.h"

#include <random>

namespace cartographer {
namespace evaluation {
namespace {
static std::default_random_engine e1(42);
}  // namespace

ScanCloudGenerator::ScanCloudGenerator(
    cartographer::sensor::PointCloud *scan_cloud)
    : cloud(scan_cloud), resolution_(0.02) {}

ScanCloudGenerator::ScanCloudGenerator(
    cartographer::sensor::PointCloud *scan_cloud, float resolution)
    : cloud(scan_cloud), resolution_(resolution) {}

void ScanCloudGenerator::generateSquare(float size, float noise_std_dev) {
  generateRectangle(size, size, noise_std_dev);
}

void ScanCloudGenerator::generateRectangle(float size_x, float size_y,
                                           float noise_std_dev) {
  // std::random_device r;
  //
  std::normal_distribution<float> normal_distribution(0,
                                                      noise_std_dev);  // 0.01

  cloud->clear();
  float x_min = -size_x / 2.0f;
  float x_max = size_x / 2.0;
  float y_min = -size_y / 2.0;
  float y_max = size_y / 2.0;

  std::uniform_real_distribution<double> error_translation_direction(-M_PI,
                                                                     M_PI);
  double e_orientation;
  double e_scale = noise_std_dev == 0.0 ? 0.0 : noise_std_dev;
  double e_x;
  double e_y;
  error_sqr = 0;
  for (float x = x_min + 0.0000f; x <= x_max - 0.0000f + 1e-5;
       x += resolution_) {
    float y = y_min;
    e_orientation = normal_distribution(e1);
    e_x = x / sqrt(x * x + y * y) * e_orientation;
    e_y = y / sqrt(x * x + y * y) * e_orientation;
    cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
    error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
                          y = y_max;
    e_orientation = normal_distribution(e1);
    e_x = x / sqrt(x * x + y * y) * e_orientation;
    e_y = y / sqrt(x * x + y * y) * e_orientation;
    cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
    error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
  }
  for (float y = y_min + 0.0000f; y <= y_max - 0.0000f + 1e-5;
       y += resolution_) {
    float x = x_min;
    e_orientation = normal_distribution(e1);
    e_x = x / sqrt(x * x + y * y) * e_orientation;
    e_y = y / sqrt(x * x + y * y) * e_orientation;
    cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
    error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
    x = x_max;
    e_orientation = normal_distribution(e1);
    e_x = x / sqrt(x * x + y * y) * e_orientation;
    e_y = y / sqrt(x * x + y * y) * e_orientation;
    cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
    error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
  }
  error_sqr = error_sqr / cloud->size();
 }

bool is_x_y_radar_valid(float x, float y) {

  float angle = atan(y / x);
  if (x < 0) return false;
  if (abs(angle) < 30 * M_PI / 180 || (abs(angle) > 55 * M_PI / 180 && abs(angle) < 83 * M_PI / 180)) return true;
  return false;
}

void ScanCloudGenerator::generateRectangleRadar(float size_x, float size_y,
                                                float noise_std_dev) {
  // std::random_device r;
  //
  std::normal_distribution<float> normal_distribution(0,
                                                      noise_std_dev);  // 0.01

  cloud->clear();
  float x_min = -size_x / 2.0f;
  float x_max = size_x / 2.0;
  float y_min = -size_y / 2.0;
  float y_max = size_y / 2.0;

  std::uniform_real_distribution<double> error_translation_direction(-M_PI,
                                                                     M_PI);
  double e_orientation;
  double e_scale = noise_std_dev == 0.0 ? 0.0 : noise_std_dev;
  double e_x;
  double e_y;
  error_sqr = 0;
  for (float x = x_min + 0.0000f; x <= x_max - 0.0000f + 1e-5;
       x += resolution_) {
    float y = y_min;
    if (is_x_y_radar_valid(x, y)) {
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
      error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
    }
    y = y_max;
    if (is_x_y_radar_valid(x, y)) {
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
      error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
    }
  }
  for (float y = y_min + 0.0000f; y <= y_max - 0.0000f + 1e-5;
       y += resolution_) {
    float x = x_min;
    if (is_x_y_radar_valid(x, y)) {
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
      error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
    }
    x = x_max;
    if (is_x_y_radar_valid(x, y)) {
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud->push_back({Eigen::Vector3f(x + e_x, y + e_y, 0.)});
      error_sqr += sqrt(pow(e_x - x, 2) + pow(e_y - y, 2));
    }
  }
  error_sqr = error_sqr / cloud->size();
}


void ScanCloudGenerator::generateCircle(float radius, float noise_std_dev) {
  // std::random_device r;
  // std::default_random_engine e1(42);
  std::normal_distribution<float> normal_distribution(0, noise_std_dev);

  cloud->clear();
  float angular_resolution = 2.0 * std::asin(resolution_ / (2 * radius));

  for (float angle = 0; angle < 2.0 * M_PI; angle += angular_resolution) {
    float x = std::cos(angle) * radius;
    float y = std::sin(angle) * radius;
    cloud->push_back({Eigen::Vector3f(x + normal_distribution(e1),
                                     y + normal_distribution(e1), 0.)});
  }
}

void ScanCloudGenerator::generateOutlier(float size_x, float size_y,
                                         float outlier_ratio,
                                         float outlier_dist=0.2,
                                         ModelType model_type=ScanCloudGenerator::ModelType::RECTANGLE) {
  int total_outlier = cloud->size() * outlier_ratio;
  int initial_size = cloud->size();
//  LOG(INFO) << "Total outlier: " << total_outlier;
  for (int n_outlier = 0; n_outlier < total_outlier; n_outlier++) {
    float current_outlier_dist = 0.0;
    Eigen::Vector3f outlier;
    while (current_outlier_dist < outlier_dist) {
      outlier = Eigen::Vector3f::Random();
      outlier.x() = outlier.x() * size_x;
      outlier.y() = outlier.y() * size_y;
      outlier.z() = 0;
      float min_dist = 1000000;
      for (int i=0; i < initial_size; i++) {
        float d = (outlier - cloud->at(i).position).norm();
        if (d < min_dist)
          min_dist = d;
      }
      if (model_type == ScanCloudGenerator::ModelType::RECTANGLE_RADAR) {
        if (!is_x_y_radar_valid(outlier.x(), outlier.y())) continue;
      }
      current_outlier_dist = min_dist;
    }
    cloud->push_back({outlier});
  }
}

void ScanCloudGenerator::removePoints(float sparse_ratio) {
  int total_remove = cloud->size() * sparse_ratio;
//  LOG(INFO) << "Total outlier: " << total_outlier;
  for (int n_remove = 0; n_remove < total_remove; n_remove++) {
    auto random_point_idx = (rand() % cloud->size()) + 1;
    cloud->erase(cloud->begin() + random_point_idx);
  }
}

void ScanCloudGenerator::generateFromCsv(const std::string & path,
                                         int start_scan, int scans_to_load) {
  std::ifstream in_data;
  in_data.open(path);
  std::string line;
//  cartographer::sensor::PointCloud cloud;
  uint rows = 0;
  int current_scan = -1;  // -1: ignore first empty line
  while (std::getline(in_data, line) &&
         current_scan - start_scan < scans_to_load) {
    std::stringstream lineStream(line);
    if (line.size() <= 1) {  // line is '\r'
      ++current_scan;
    } else {
      if (start_scan <= current_scan) {
        std::string cell;
        std::vector<double> temp(3);
        uint c = 0;
        while (std::getline(lineStream, cell, ',')) {
          temp.at(c) = std::stod(cell);
          ++c;
        }
        Eigen::Vector3f new_point(temp[0], temp[1], temp[2]);
//        Eigen::Vector3f new_point = Eigen::Map<Eigen::Vector3f>(temp.data(),
//                                                                temp.size());
        cloud->push_back({new_point});
        ++rows;
      }
    }
  }
//  return cloud;
}

Eigen::Vector3f ScanCloudGenerator::getStdDev() {
  double mean_x = 0;
  double mean_y = 0;
  double std_dev_x = 0;
  double std_dev_y = 0;
  double std_dev_xy = 0;
  double n = 0;
  n = n + cloud->size();
  for (auto point : *cloud) {
    mean_x += point.position.x();
    mean_x += point.position.y();
  }
  mean_x = mean_x / n;
  mean_y = mean_y / n;
  for (auto point : *cloud) {
    std_dev_x += pow(point.position.x() - mean_x, 2);
    std_dev_y += pow(point.position.y() - mean_y, 2);
    std_dev_xy += (point.position.x() - mean_x) *
                  (point.position.y() - mean_y);
  }
  std_dev_x = std_dev_x / n;
  std_dev_y = std_dev_y / n;
  std_dev_xy = std_dev_xy / n;
  return Eigen::Vector3f(std_dev_x, std_dev_y, std_dev_xy);
}

}  // namespace evaluation
}  // namespace cartographer
