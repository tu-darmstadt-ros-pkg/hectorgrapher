//
// Created by bhirschel on 20.02.21.
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

class DynamicObjectsRemovalPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char *kConfigurationFileActionName =
      "dynamic_objects_removal_filter";
  sensor::CustomPointCloud map_; // Cannot be made private to use it in test
  enum class RunState {
    kInitialRun,
    kSecondRun,
  };
  static RunState run_state_;

  /**
   * Points Processor that deletes dynamic objects' spurious point detections
   * from the sequence of points batches in the bag file. It is based on
   * voxelization on spherical coordinates. This implementation was developed
   * in the course of an "Integriertes Projekt Robotik" at SIM TU Darmstadt in
   * 2020/21. Details are to be found in the report "Dynamic Object Filtering
   * for 3D Model Generation with Mobile Ground Robots".
   * @warning This module should be inserted into the pipeline as one of the
   * first, since it requires the pipeline to restart once and only sends the
   * modified points batches during the second run!
   * @param r_segments Number of range segments to be generated within
   * [0, sensor_range_limit]
   * @param theta_segments Number of theta (elevation angle) segments to be
   * generated in [0, pi]
   * @param phi_segments Number of phi (azimuthal angle) segments to be
   * generated in [0, 2pi]
   * @param sensor_range_limit Limit up to which dynamic objects are filtered [m]
   * @param probability_reduction_factor Amount that a point classified as
   * dynamic gets devalued
   * @param dynamic_object_probability_threshold Threshold up to which each
   * point is considered to belong to a static object. If a point's probability
   * falls below this threshold, it will be discarded from the global map
   * @param search_ray_threshold Percentage up to how far in the search ray
   * from the origin towards the most significant wedge, wedges get devalued
   * @param open_view_deletion Optional boolean value to enable open view
   * deletion targeting the "Point at Infinity" problem. Note that enabling this
   * may severely affect runtime and the quality of the result
   * @param show_extended_debug_information Optional boolean value to show
   * extended command line output regarding the algorithm's performance
   * @param next Interlinking pointer to the next PointsProcessor
   */
  DynamicObjectsRemovalPointsProcessor(int r_segments,
                                       int theta_segments,
                                       int phi_segments,
                                       double sensor_range_limit,
                                       double probability_reduction_factor,
                                       double dynamic_object_probability_threshold,
                                       double search_ray_threshold,
                                       bool open_view_deletion,
                                       bool show_extended_debug_information,
                                       PointsProcessor *next);

  static std::unique_ptr<DynamicObjectsRemovalPointsProcessor> FromDictionary(
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~DynamicObjectsRemovalPointsProcessor() override {}

  DynamicObjectsRemovalPointsProcessor(
      const DynamicObjectsRemovalPointsProcessor &) = delete;
  DynamicObjectsRemovalPointsProcessor &operator=(
      const DynamicObjectsRemovalPointsProcessor &) = delete;

  /**
   * Main routine to process a points batch
   * @param batch points batch structure containing data from the scan such as
   * the pointcloud or the transformation.
   */
  void Process(std::unique_ptr<PointsBatch> batch) override;

  /**
   * Handling of the flush signal. During the initial run, it will flush the
   * points to the batch, while during the second run it will only print debug
   * information if enabled
   * @return
   */
  FlushResult Flush() override;

 private:
  const int r_segments_, theta_segments_, phi_segments_;
  const double sensor_range_limit_, probability_reduction_factor_,
      dynamic_object_probability_threshold_, search_ray_threshold_;
  const bool open_view_deletion_;
  const bool show_extended_debug_information_;
  PointsProcessor *const next_;

  std::vector<PointsBatch> list_of_batches_;
  transform::Rigid3<float> robot_translation_;
  uint16_t scan_batch_max_range_;
  int iteration_ = -1;

  // Evaluation measurements
  size_t eval_total_points_;
  std::chrono::system_clock::time_point eval_total_time_begin_;
  std::chrono::milliseconds eval_total_time_elapsed_;
  std::vector<std::chrono::milliseconds> eval_time_detailed_;
  std::vector<size_t> eval_cumulated_number_of_points_,
      eval_number_deleted_points_;

  /**
   * Transforming a cartesian point p = (x, y, z) into polar coordinate
   * p_polar = (r, theta, phi)
   * @param cart_coord Point in Cartesian coordinates to be transformed to polar
   * @return polar coordinate representation
   */
  static Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f cart_coord);

  /**
   * Transforming a polar point p_polar (r, theta, phi) into Cartesian
   * p = (x, y, z)
   * @param r Range value
   * @param theta Theta value
   * @param phi Phi value
   * @return Cartesian vector p
   */
  static Eigen::Vector3f polar_to_cartesian(float r, float theta, float phi);

  /**
   * Cantor pairing function that maps two natural positive values a and b onto
   * a unique value c: c= <a, b>
   * @param a first value
   * @param b second value
   * @return c unique mapping result
   */
  static uint16_t cantor_pairing(uint16_t a, uint16_t b);

  /**
   * Key structure for the wedge map, composed of three values p (range), theta
   * and phi
   */
  typedef std::tuple<uint16_t, uint16_t, uint16_t> wedge_key_t;

  /**
   * Hashing function that, given a wedge_key, uniquely pairs its' three values
   * onto a single new value using the Cantor pairing function, so that it can
   * be used to index a hash map structure
   */
  struct key_hash : public std::unary_function<key_t, std::size_t> {
    std::size_t operator()(const wedge_key_t &k) const {
      return cantor_pairing(cantor_pairing(std::get<0>(k), std::get<1>(k)),
                            std::get<2>(k));
    }
  };

  /**
   * Key equal function checking whether two provided wedge keys are equal. The
   * keys are considered equal, if all three values p (range), theta and phi are
   * equal
   */
  struct key_equal : public std::binary_function<wedge_key_t,
                                                 wedge_key_t,
                                                 bool> {
    bool operator()(const wedge_key_t &k1, const wedge_key_t &k2) const {
      return (
          std::get<0>(k1) == std::get<0>(k2) &&
              std::get<1>(k1) == std::get<1>(k2) &&
              std::get<2>(k1) == std::get<2>(k2)
      );
    }
  };

  /**
   * Spherical wedge is s structure originating from polar coordinate space,
   * that holds exactly one CustomPointCloud, which contains all the points
   * belonging to this wedge
   */
  struct sphercial_wedge {
    sensor::CustomPointCloud wedge_points;
  };

  /**
   * The wedge map is an unordered map structure that stores spherical wedges
   * and is accessed by a wedge key
   */
  typedef std::unordered_map<const wedge_key_t,
                             sphercial_wedge,
                             key_hash,
                             key_equal> wedge_map_t;

  /**
   * Hashing function that, given a key, uniquely pairs its' two values
   * onto a single new value using the Cantor pairing function, so that it can
   * be used to index a hash map structure. This is required for scan wedge
   * cardinalities
   */
  struct key_hash_pair : public std::unary_function<key_t, std::size_t> {
    std::size_t operator()(const std::pair<uint16_t, uint16_t> &k) const {
      return cantor_pairing(std::get<0>(k), std::get<1>(k));
    }
  };

  /**
   * Key equal function checking whether two provided keys are equal. The keys
   * are considered equal, if all two values are equal
   */
  struct key_equal_pair : public std::binary_function<std::pair<uint16_t,
                                                                uint16_t>,
                                                      std::pair<uint16_t,
                                                                uint16_t>,
                                                      bool> {
    bool operator()(const std::pair<uint16_t, uint16_t> &k1,
                    const std::pair<uint16_t, uint16_t> &k2) const {
      return (
          std::get<0>(k1) == std::get<0>(k2)
              && std::get<1>(k1) == std::get<1>(k2)
      );
    }
  };

  /**
   * Given a batch, this function initializes a custom point cloud with the
   * position, color and intensity from the batch, 1.0 as initial probability
   * and the index from the associated index parameter. Color and intensity are
   * set to NAN if they are not specified in the batch. The CustomPointCloud
   * will be written at the position of the pointer given by scan_map
   * @param scan_map pointer to a CustomPointCloud to save the initialized
   * pointcloud
   * @param batch PointsBatch from which to take the data
   * @param index of the batch in the pipeline
   */
  void initialize_scan_map(sensor::CustomPointCloud &scan_map,
                           PointsBatch *batch,
                           int index);

  /**
   * Forces a 3D vector of polar coordinates into an interval defined by the
   * number of segments for r, theta and phi.
   * @return Returns the index of those segments as tuple
   */
  wedge_key_t get_interval_segment(Eigen::Vector3f) const;

  /**
   * Allocates every point from the given pointcloud to its associated wedge and
   * returns the full wedge map. Sets the class variable scan_batch_max_range_
   * according to the highest range segment found within the scan batch
   * @param cloud The pointcloud as sensor::ProbabilityIndexedPointCloud
   * @param is_scan_batch True if the pointcloud is from the local scan batch
   * @return A wedge map as wedge_map_t
   */
  wedge_map_t create_wedge_map(const sensor::CustomPointCloud &cloud,
                               bool is_scan_batch);

  /**
   * Iterates through the global map and reconstructs points batches from it,
   * according to the points' index value. It carries the points 3D position
   * vector, intensities and colors over to the associated structures in the
   * points batch
   */
  void flush_points_to_batch();
};

}  // namespace io
}  // namespace cartographer
