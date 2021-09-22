//
// Created by ubuntu on 20.02.21.
//

#include <memory>
#include <Eigen/Dense>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/io/file_writer.h"

#define M_PIf static_cast<float>(M_PI)

namespace cartographer {
namespace io {

//TODO(bhirschel) here objectS different to filename
class DynamicObjectsRemovalPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char *kConfigurationFileActionName =
      "dynamic_objects_removal_filter";
  sensor::CustomPointCloud map_;
  enum class RunState {
    kInitialRun,
    kSecondRun,
  };
  static RunState run_state_;

  DynamicObjectsRemovalPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                                       int r_segments,
                                       int theta_segments,
                                       int phi_segments,
                                       double sensor_range_limit,
                                       int end_of_file,
                                       double probability_reduction_factor,
                                       double dynamic_object_probability_threshold,
                                       double search_ray_threshold,
                                       bool open_view_deletion,
                                       PointsProcessor *next);

  static std::unique_ptr<DynamicObjectsRemovalPointsProcessor> FromDictionary(
      const FileWriterFactory &file_writer_factory,
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~DynamicObjectsRemovalPointsProcessor() override {}

  DynamicObjectsRemovalPointsProcessor(
      const DynamicObjectsRemovalPointsProcessor &) = delete;
  DynamicObjectsRemovalPointsProcessor &operator=(
      const DynamicObjectsRemovalPointsProcessor &) = delete;

  static Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f cart_coord);
  static Eigen::Vector3f polar_to_cartesian(float r, float theta, float phi);

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const int r_segments_, theta_segments_, phi_segments_, end_of_file_;
  const double sensor_range_limit_, probability_reduction_factor_,
      dynamic_object_probability_threshold_, search_ray_threshold_;
  const bool open_view_deletion_;
  std::vector<PointsBatch> list_of_batches_;
  PointsProcessor *const next_;
  std::unique_ptr<FileWriter> file_;
  transform::Rigid3<float> robot_translation_;
  uint16_t scan_batch_max_range_;
  int iteration_ = -1;

  // Evaluation measurements
  size_t eval_total_points_;
  std::chrono::system_clock::time_point eval_total_time_begin_;
  std::chrono::milliseconds eval_total_time_elapsed_;
  std::vector<std::chrono::milliseconds> eval_time_detailed_;

  static uint16_t cantor_pairing(uint16_t a, uint16_t b);

  typedef std::tuple<uint16_t, uint16_t, uint16_t> wedge_key_t;

  struct key_hash : public std::unary_function<key_t, std::size_t> {
    std::size_t operator()(const wedge_key_t &k) const {
      // Use cantor pairing function to generate unique hash
      return cantor_pairing(cantor_pairing(std::get<0>(k), std::get<1>(k)), std::get<2>(k));
    }
  };

  struct key_equal : public std::binary_function<wedge_key_t, wedge_key_t, bool> {
    bool operator()(const wedge_key_t &k1, const wedge_key_t &k2) const {
      // Only equal if all three key values are exactly equal
      return (
          std::get<0>(k1) == std::get<0>(k2) &&
              std::get<1>(k1) == std::get<1>(k2) &&
              std::get<2>(k1) == std::get<2>(k2)
      );
    }
  };

  struct sphercial_wedge {
    sensor::CustomPointCloud wedge_points;
  };

  typedef std::unordered_map<const wedge_key_t, sphercial_wedge, key_hash, key_equal> wedge_map_t;

  struct key_hash_pair : public std::unary_function<key_t, std::size_t> {
    std::size_t operator()(const std::pair<uint16_t, uint16_t> &k) const {
      // Use cantor pairing function to generate unique hash
      return cantor_pairing(std::get<0>(k), std::get<1>(k));
    }
  };

  struct key_equal_pair : public std::binary_function<std::pair<uint16_t, uint16_t>,
                                                      std::pair<uint16_t, uint16_t>,
                                                      bool> {
    bool operator()(const std::pair<uint16_t, uint16_t> &k1,
                    const std::pair<uint16_t, uint16_t> &k2) const {
      // Only equal if both key values are exactly equal
      return (
          std::get<0>(k1) == std::get<0>(k2) && std::get<1>(k1) == std::get<1>(k2)
      );
    }
  };

  /**
   * Given a batch, this function initializes a custom point cloud with the position, color and
   * intensity from the batch, 1.0 as initial probability and the index from the associated index
   * parameter. Color and intensity are set to NAN if they are not specified in the batch. The
   * CustomPointCloud will be written at the position of the pointer given by scan_map
   * @param scan_map pointer to a CustomPointCloud to save the initialized pointcloud
   * @param batch PointsBatch from which to take the data
   * @param index of the batch in the pipeline
   */
  void initialize_scan_map(sensor::CustomPointCloud &scan_map, PointsBatch *batch, int index);

  /**
   * Forces a 3D vector of polar coordinates into an interval defined by the number of segments for
   * r, theta and phi.
   * @return Returns the index of those segments as tuple
   */
  wedge_key_t get_interval_segment(Eigen::Vector3f) const;

  /**
   * Allocates every point from the given pointcloud to its associated wedge and returns the full
   * wedge map. Sets the class variable scan_batch_max_range_ according to the highest range segment
   * found within the scan batch
   * @param cloud The pointcloud as sensor::ProbabilityIndexedPointCloud
   * @param is_scan_batch True if the pointcloud is from the local scan batch
   * @return A wedge map as wedge_map_t
   */
  wedge_map_t create_wedge_map(const sensor::CustomPointCloud &cloud, bool is_scan_batch);

  void flush_points_to_batch();
};

}  // namespace io
}  // namespace cartographer
