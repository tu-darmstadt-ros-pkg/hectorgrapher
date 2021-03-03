//
// Created by ubuntu on 20.02.21.
//

#include <memory>
#include <Eigen/Dense>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/sensor/point_cloud.h"

#define M_PIf static_cast<float>(M_PI)

namespace cartographer {
namespace io {

class DynamicObjectsRemovalPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "dynamic_objects_removal_filter";
  DynamicObjectsRemovalPointsProcessor(PointsProcessor* next);

  static std::unique_ptr<DynamicObjectsRemovalPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~DynamicObjectsRemovalPointsProcessor() override {}

  DynamicObjectsRemovalPointsProcessor(
      const DynamicObjectsRemovalPointsProcessor&) = delete;
  DynamicObjectsRemovalPointsProcessor& operator=(
      const DynamicObjectsRemovalPointsProcessor&) = delete;

  Eigen::Vector3f cartesian_to_polar(Eigen::Vector3f cart_coord);

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  float r_segments_, theta_segments_, phi_segments_, sensor_range_limit_;
  sensor::PointCloud map_;
  std::vector<PointsBatch> list_of_batches_;
  PointsProcessor* const next_;

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
};

}  // namespace io
}  // namespace cartographer
