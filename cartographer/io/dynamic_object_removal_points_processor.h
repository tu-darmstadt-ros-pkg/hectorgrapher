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
  constexpr static const char* kConfigurationFileActionName =
      "dynamic_objects_removal_filter";
  sensor::PointCloud map_;
  enum class RunState {
    kInitialRun,
    kSecondRun,
  };
  static RunState run_state_;

  DynamicObjectsRemovalPointsProcessor(std::unique_ptr<FileWriter> file_writer,
      int r_segments, int theta_segments, int phi_segments, float sensor_range_limit,
      int end_of_file, PointsProcessor* next);

  static std::unique_ptr<DynamicObjectsRemovalPointsProcessor> FromDictionary(
      const FileWriterFactory& file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~DynamicObjectsRemovalPointsProcessor() override {}

  DynamicObjectsRemovalPointsProcessor(
      const DynamicObjectsRemovalPointsProcessor&) = delete;
  DynamicObjectsRemovalPointsProcessor& operator=(
      const DynamicObjectsRemovalPointsProcessor&) = delete;

  Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f cart_coord);
  Eigen::Vector3f polar_to_cartesian(float r, float theta, float phi);

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const int r_segments_, theta_segments_, phi_segments_, end_of_file_;
  const double sensor_range_limit_;
  std::vector<PointsBatch> list_of_batches_;
  PointsProcessor* const next_;
  std::unique_ptr<FileWriter> file_;
  transform::Rigid3<float> sensor_height_adjustment_;

  void initialize_probabilities(PointsBatch &batch);

  static uint16_t cantor_pairing(uint16_t a, uint16_t b);

  typedef std::tuple<uint16_t, uint16_t, uint16_t> wedge_key_t;

  struct key_hash : public std::unary_function<key_t, std::size_t>
  {
    std::size_t operator()(const wedge_key_t& k) const
    {
      // Use cantor pairing function to generate unique hash
      return cantor_pairing(cantor_pairing(std::get<0>(k), std::get<1>(k)), std::get<2>(k));
    }
  };

  struct key_equal : public std::binary_function<wedge_key_t, wedge_key_t, bool>
  {
    bool operator()(const wedge_key_t& k1, const wedge_key_t& k2) const
    {
      // Only equal if all three key values are exactly equal
      return (
          std::get<0>(k1) == std::get<0>(k2) &&
          std::get<1>(k1) == std::get<1>(k2) &&
          std::get<2>(k1) == std::get<2>(k2)
      );
    }
  };

  struct sphercial_wedge {
    //TODO(bastian.hirschel) don't need the float fields min_r etc, can retrieve that from the key
    float min_r, max_r, min_theta, max_theta, min_phi, max_phi;
    sensor::PointCloud wedge_points;
  };

  typedef std::unordered_map<const wedge_key_t,sphercial_wedge,key_hash,key_equal> wedge_map_t;

  /**
   * Forces a 3D vector of polar coordinates into an interval defined by the number of segments for
   * r, theta and phi.
   * @return Returns the index of those segments as tuple
   */
  wedge_key_t get_interval_segment(Eigen::Vector3f);

  wedge_map_t create_wedge_map(sensor::PointCloud cloud);

  /**
   * Removes a point from a pointcloud if he is associated to a wedge with the given key
   * @param key as 3-tuple of wedge_key_t
   * @param cloud as sensor::PointCloud
   */
  void remove_points_from_pointcloud(std::vector<wedge_key_t> keys_to_delete,
                                     sensor::PointCloud &cloud,
                                     transform::Rigid3<float> transformation);
  size_t remove_points_from_batch(std::vector<wedge_key_t> keys_to_delete,
                                PointsBatch &batch,
                                transform::Rigid3<float> transformation);

  //void write_random_wedge_to_file(float r, float theta, float phi);
};

}  // namespace io
}  // namespace cartographer
