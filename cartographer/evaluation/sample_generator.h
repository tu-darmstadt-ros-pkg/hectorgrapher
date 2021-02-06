#ifndef CARTOGRAPHER_SAMPLE_GENERATOR_H
#define CARTOGRAPHER_SAMPLE_GENERATOR_H

#include <cartographer/sensor/point_cloud.h>
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/evaluation/scan_cloud_generator.h"
#include <random>
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"

namespace cartographer {
namespace evaluation {

struct Sample {
  cartographer::sensor::RangeData range_data;
  cartographer::transform::Rigid2d ground_truth_pose;
  double std_dev_x;
  double std_dev_y;
  double std_dev_xy;
  double error_sqr;
};

struct SampleResult {
  double initial_trans_error;
  double initial_rot_error;
  double matching_trans_error;
  double matching_rot_error;
  double matching_time;
  int matching_iterations;
};


static std::random_device rd;
static std::default_random_engine e1(42);

class SampleGenerator {
public:
  SampleGenerator(int n_training, int n_test, double error_trans,
                  double error_rot, float outlier_ratio);
  SampleGenerator(int n_training, int n_test, double error_trans,
                  double error_rot, float outlier_ratio, float sparse_ratio);

  void GenerateSampleSet(ScanCloudGenerator::ModelType model_type,
                         Eigen::Vector2d &size, double resolution,
                         float cloud_noise_std_dev, float outlier_training);
  void GenerateSampleSet(std::string &csv_file, int start_scan,
                         int scans_to_load);

  void GenerateDefinedSampleSet(ScanCloudGenerator::ModelType model_type,
                                Eigen::Vector2d &size, double resolution,
                                float cloud_noise_std_dev);

  Sample GenerateSample(double error_trans_, double error_rot_,
                        const ScanCloudGenerator::ModelType model_type,
                        Eigen::Vector2d &size, const double resolution,
                        float cloud_noise_std_dev, float outlier_ratio);

  Sample GenerateSample(double error_trans_, double error_rot_,
                        std::string &csv_file, int start_scan, int scans_to_load);

  Sample GenerateSample(Eigen::Vector3f single_point);
  Sample GenerateSample(std::vector<Eigen::Vector3f> &points);

  std::vector<Sample> getTrainingSet() { return training_set; }
  std::vector<Sample> getTestSet() { return test_set; }
  double std_dev_x;
  double std_dev_y;
  double std_dev_xy;
  double error_sqr;


private:
//  void GenerateRangeDataCommon(ScanCloudGenerator &test_set_generator);

  void GenerateRangeData(ScanCloudGenerator::ModelType model_type,
                         Eigen::Vector2d &size, double resolution,
                         cartographer::sensor::RangeData &range_data,
                         float noise_std_dev, float outlier_ratio);

  void GenerateRangeData(std::string &csv_file, int start_scan, int scans_to_load,
                         cartographer::sensor::RangeData &range_data) const;

  Sample GenerateDefinedSample(double error_x, double error_y, double error_phi,
                               ScanCloudGenerator::ModelType model_type,
                               Eigen::Vector2d &size, double resolution,
                               float cloud_noise_std_dev);

  Sample PerturbateRangeData(double error_trans_, double error_rot_,
                                    cartographer::sensor::RangeData& range_data);

  int n_training;
  int n_test;
  double error_trans;
  double error_rot;
  float outlier_ratio;
  float sparse_ratio;
  std::vector<Sample> training_set;
  std::vector<Sample> test_set;
};

}
}
#endif //CARTOGRAPHER_SAMPLE_GENERATOR_H
