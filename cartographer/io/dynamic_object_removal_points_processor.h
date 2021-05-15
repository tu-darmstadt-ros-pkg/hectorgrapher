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
  sensor::TimedPointCloud map_;
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
      dynamic_object_probability_threshold_;
  const bool open_view_deletion_;
  std::vector<PointsBatch> list_of_batches_;
  PointsProcessor *const next_;
  std::unique_ptr<FileWriter> file_;
  transform::Rigid3<float> sensor_height_adjustment_;
  uint16_t scan_batch_max_range_;

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
    sensor::TimedPointCloud wedge_points;
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
   * Forces a 3D vector of polar coordinates into an interval defined by the number of segments for
   * r, theta and phi.
   * @return Returns the index of those segments as tuple
   */
  wedge_key_t get_interval_segment(Eigen::Vector3f) const;

  /**
   * Allocates every point from the given pointcloud to its associated wedge and returns the full
   * wedge map. When allocating the global wedge map, a range limit defined by sensor_range_limit_
   * is enforced, while for a local scan every point is added to the wedge map
   * @param cloud The pointcloud as sensor::TimedPointCloud
   * @param is_scan_batch True if the pointcloud is from the local scan batch. Toggles enforcement
   * of the sensor range limit
   * @return A wedge map as wedge_map_t
   */
  wedge_map_t create_wedge_map(const sensor::TimedPointCloud &cloud, bool is_scan_batch);

  /**
   * Given a vector of wedge map keys that are identified as dynamic object points and a points
   * batch, this function reduces their probability by the global factor
   * probability_reduction_factor_. If their resulting probability is below the threshold defined by
   * dynamic_object_probability_threshold_, they are deleted using the safe method provided by
   * DynamicObjectsRemovalPointsProcessor::RemoveTimedPoints()
   * @param keys_to_lower std vector of wedge_key_t containing wedge map keys belonging to dynamic
   * object detections
   * @param batch reference to the PointsBatch to work on
   * @param transformation as transform::Rigid3<float> to be applied to each point before checking
   * its associated wedge key
   * @return total number of points that have been lowered in their probability
   */
  size_t lower_prob_in_batch(std::vector<wedge_key_t> keys_to_lower,
                             PointsBatch &batch,
                             const transform::Rigid3<float> &transformation);

  /**
   * Safe removal of timed points from a PointsBatch. Deletes the points_with_probabilities from the
   * batch that have their index in the given list to_remove. Also removes associated intensities
   * and color entries, if given.
   * @param to_remove list of indices of points_with_probabilities to be removed
   * @param batch reference to the PointsBatch to work on
   */
  static void RemoveTimedPoints(const absl::flat_hash_set<int> &to_remove, PointsBatch *batch);

  /**
   * Given a PointsBatch, this function copies all original points of type sensor::RangefinderPoint
   * to the new list points_with_probabilities of type sensor::TimedRangefinderPoint. The additional
   * time field of a TimedRangefinderPoint is used to store the points probability to be a detection
   * of a static object and is initialized to 1.0. Clears points.
   * @param batch reference to the PointsBatch to work on
   */
  static void initialize_probabilities(PointsBatch &batch);

  /**
   * Inverse operation to ::initialize_probabilities. Copies all points_with_probabilities of type
   * sensor::TimedRangefinderPoint back to their original field points of type
   * sensor::RangefinderPoint. Clears points_with_probabilities.
   * @param batch reference to the PointsBatch to work on
   */
  static void copy_points_to_batch(PointsBatch &batch);
};

}  // namespace io
}  // namespace cartographer
