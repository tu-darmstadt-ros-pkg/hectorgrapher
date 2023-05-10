
#ifndef CARTOGRAPHER_MAPPING_3D_STOP_WATCH_H_
#define CARTOGRAPHER_MAPPING_3D_STOP_WATCH_H_

#include <chrono>
#include <map>
#include <string>

#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

class StopWatch {
 public:
  void Start();
  void Stop();
  double last_duration_ = 0.0;
  double summed_duration_ = 0.0;
  size_t num_observations_ = 0;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

class StopWatchManger {
 public:
  ~StopWatchManger();
  StopWatch& GetWatch(const std::string& name);

  void PrintAll();
  void PrintAllEveryN(size_t n);

 private:
  std::map<std::string, StopWatch> watches_;
  size_t request_since_last_print_ = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_STOP_WATCH_H_
