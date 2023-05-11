#ifndef CARTOGRAPHER_SENSOR_INTERNAL_SAMPLING_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_SAMPLING_FILTER_H_

#include <bitset>

#include "absl/container/flat_hash_set.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {

// Downsamples a pointcloud such that "ratio" portion of the pointcloud will be kept.
template<typename PointCloudType>
PointCloudType SamplingFilter(const PointCloudType &point_cloud, float ratio) {
  CHECK_LE(ratio, 1.0);
  CHECK_GE(ratio, 0.0);
  PointCloudType results;

  size_t num_insertions = 0;
  size_t num_total = 0;
  for (const auto &point : point_cloud) {
    if (num_total == 0 || float(num_insertions) / float(num_total) < ratio) {
      results.push_back(point);
      num_insertions++;
    }
    num_total++;
  }
  return results;
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_SAMPLING_FILTER_H_
