#include "cartographer/evaluation/sample_generator.h"

namespace cartographer {
namespace evaluation {

void SampleGenerator::GenerateDefinedSampleSet(
    const ScanCloudGenerator::ModelType model_type, Eigen::Vector2d &size,
    const double resolution, const float cloud_noise_std_dev) {
  for (int i = 0; i < n_training; ++i) {
    training_set.push_back(
        GenerateDefinedSample(0.0, 0.0, 0.0,
                              model_type, size, resolution,
                              cloud_noise_std_dev));
  }

  double error_scale = 0.5 / n_test;
  for (int i_dx = -n_test; i_dx <= n_test; ++i_dx) {
    for (int i_dy = -n_test; i_dy <= n_test; ++i_dy) {
      test_set.push_back(
        GenerateDefinedSample(i_dx * error_scale,
                              i_dy * error_scale, 0.0,
                              model_type, size, resolution,
                              cloud_noise_std_dev));
    }
  }
}

void SampleGenerator::GenerateSampleSet(
    const ScanCloudGenerator::ModelType model_type, Eigen::Vector2d &size,
    const double resolution, const float cloud_noise_std_dev,
    float outlier_training=0.0) {
  test_set.clear();
  training_set.clear();
  for (int i = 0; i < n_training; ++i) {
    training_set.push_back(GenerateSample(0.0, 0.0, model_type, size,
                                          resolution, cloud_noise_std_dev,
                                          outlier_training));
  }
  for (int i = 0; i < n_test; ++i) {
    test_set.push_back(GenerateSample(error_trans, error_rot, model_type, size,
                                      resolution, cloud_noise_std_dev, outlier_ratio));
  }
}
void SampleGenerator::GenerateSampleSet(
    std::string &csv_file, int start_scan, int scans_to_load) {
  for (int i = 0; i < n_training; ++i) {
    training_set.push_back(
        GenerateSample(0.0, 0.0, csv_file, start_scan,
                       scans_to_load));
  }
  for (int i = 0; i < n_test; ++i) {
    test_set.push_back(
        GenerateSample(error_trans, error_rot, csv_file, start_scan,
                       scans_to_load));
  }
}

//void SampleGenerator::GenerateRangeDataCommon(
//    ScanCloudGenerator &test_set_generator) {
//  if ((bool)outlier_ratio) {
//    test_set_generator.generateOutlier(scan_cloud, size[0], size[1],
//                                       outlier_ratio);
//  }
//  range_data.returns = scan_cloud;
//  range_data.origin = Eigen::Vector3f{0, 0, 0};
//}

void SampleGenerator::GenerateRangeData(
    const ScanCloudGenerator::ModelType model_type,
    Eigen::Vector2d &size, const double resolution,
    cartographer::sensor::RangeData &range_data, float noise_std_dev,
    float outlier_ratio) {
  ScanCloudGenerator test_set_generator(&range_data.returns, (float)resolution);
  switch (model_type) {
    case ScanCloudGenerator::ModelType::CIRCLE:
      break;
      test_set_generator.generateCircle(size[0], noise_std_dev);
    case ScanCloudGenerator::ModelType::SQUARE:
      break;
      test_set_generator.generateSquare(size[0], noise_std_dev);
    case ScanCloudGenerator::ModelType::RECTANGLE:
      test_set_generator.generateRectangle(size[0], size[1],
                                           noise_std_dev);
      break;
    case ScanCloudGenerator::ModelType::RECTANGLE_RADAR:
      test_set_generator.generateRectangleRadar(size[0], size[1],
                                                noise_std_dev);
      break;
  }
  auto std_dev = test_set_generator.getStdDev();  // calc only for inlier!
  if ((bool)sparse_ratio) {
    test_set_generator.removePoints(sparse_ratio);
  }
  if ((bool)outlier_ratio) {
    test_set_generator.generateOutlier(size[0], size[1],
                                       outlier_ratio, 0.2, model_type);
  }
//  range_data.returns = test_set_generator.getCloud();
  error_sqr = test_set_generator.error_sqr;
  std_dev_x = std_dev.x();
  std_dev_y = std_dev.y();
  std_dev_xy = std_dev.z();

  range_data.origin = Eigen::Vector3f{0, 0, 0};
}

void SampleGenerator::GenerateRangeData(
    std::string &csv_file, int start_scan, int scans_to_load,
    cartographer::sensor::RangeData &range_data) const {
  ScanCloudGenerator test_set_generator(&range_data.returns, 0.0);
  test_set_generator.generateFromCsv(csv_file, start_scan, scans_to_load);
  if ((bool)outlier_ratio) {
    // TODO get max and min sizes
//    test_set_generator.generateOutlier(scan_cloud, size[0], size[1],
//                                       outlier_ratio);
  }
  range_data.origin = Eigen::Vector3f{0, 0, 0};
}

Sample SampleGenerator::GenerateSample(double error_trans_, double error_rot_,
                                       const ScanCloudGenerator::ModelType model_type,
                                       Eigen::Vector2d &size,
                                       const double resolution,
                                       float cloud_noise_std_dev,
                                       float outlier_ratio) {
  cartographer::sensor::RangeData range_data;
  GenerateRangeData(model_type, size, resolution, range_data,
                    cloud_noise_std_dev, outlier_ratio);
  return PerturbateRangeData(error_trans_, error_rot_, range_data);
}

Sample SampleGenerator::GenerateSample(Eigen::Vector3f single_point) {
  cartographer::sensor::RangeData range_data;
  range_data.origin = Eigen::Vector3f{0, 0, 0};
  range_data.returns.push_back({ single_point });
  return PerturbateRangeData(0.0, 0.0, range_data);
}

Sample SampleGenerator::GenerateSample(std::vector<Eigen::Vector3f> &points) {
  cartographer::sensor::RangeData range_data;
  range_data.origin = Eigen::Vector3f{0, 0, 0};
  for (auto point : points)
    range_data.returns.push_back({ point });
  return PerturbateRangeData(0.0, 0.0, range_data);
}

Sample SampleGenerator::GenerateSample(double error_trans_, double error_rot_,
                                       std::string &csv_file, int start_scan,
                                       int scans_to_load) {
  cartographer::sensor::RangeData range_data;
  GenerateRangeData(csv_file, start_scan, scans_to_load, range_data);
  return PerturbateRangeData(error_trans_, error_rot_, range_data);
}

Sample SampleGenerator::PerturbateRangeData(
    double error_trans_, double error_rot_,
    cartographer::sensor::RangeData& range_data) {
  std::uniform_real_distribution<double> error_translation_direction(-M_PI, M_PI);
  double orientation = error_translation_direction(e1);
  double scale = error_trans_ == 0.0 ? 0.0 : error_trans_;
  double x = std::cos(orientation) * scale;
  double y = std::sin(orientation) * scale;
  Sample sample;
  sample.std_dev_x = std_dev_x;
  sample.std_dev_y = std_dev_y;
  sample.std_dev_xy = std_dev_xy;
  sample.error_sqr = error_sqr;
  sample.ground_truth_pose = cartographer::transform::Rigid2d({x, y}, error_rot_);
  cartographer::sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, cartographer::transform::Embed3D(
              sample.ground_truth_pose.cast<float>()));
  sample.range_data = initial_pose_estimate_range_data;
  return sample;
}

Sample SampleGenerator::GenerateDefinedSample(
    double error_x, double error_y, double error_phi,
    const ScanCloudGenerator::ModelType model_type, Eigen::Vector2d &size,
    const double resolution, float cloud_noise_std_dev) {
  cartographer::sensor::RangeData range_data;
  SampleGenerator::GenerateRangeData(model_type, size, resolution,
                                     range_data, cloud_noise_std_dev, 0.0);
  double x = error_x;
  double y = error_y;
  Sample sample;
  sample.ground_truth_pose = cartographer::transform::Rigid2d({x, y},
                                                              error_phi);
  cartographer::sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, cartographer::transform::Embed3D(
              sample.ground_truth_pose.cast<float>()));
  sample.range_data = initial_pose_estimate_range_data;
  return sample;
}

SampleGenerator::SampleGenerator(int n_training, int n_test, double error_trans,
                                 double error_rot, float outlier_ratio) : 
    n_training(n_training), n_test(n_test), error_trans( error_trans),
    error_rot(error_rot), outlier_ratio(outlier_ratio), sparse_ratio(0) {}

SampleGenerator::SampleGenerator(int n_training, int n_test, double error_trans,
                                 double error_rot, float outlier_ratio,
                                 float sparse_ratio) :
    n_training(n_training), n_test(n_test), error_trans( error_trans),
    error_rot(error_rot), outlier_ratio(outlier_ratio),
    sparse_ratio(sparse_ratio) {}

} // namespace evaluation
} // namespace cartographer