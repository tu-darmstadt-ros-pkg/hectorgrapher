
#include "stop_watch.h"

#include <iomanip>

#include "glog/logging.h"

namespace cartographer {
namespace metrics {
void StopWatch::Start() { start_time_ = std::chrono::steady_clock::now(); }
void StopWatch::Stop() {
  auto end_time = std::chrono::steady_clock::now();
  last_duration_ = std::chrono::duration_cast<std::chrono::duration<double>>(
                       end_time - start_time_)
                       .count();
  summed_duration_ += last_duration_;
  num_observations_++;
}

StopWatch& StopWatchManger::GetWatch(const std::string& name) {
  if (watches_.find(name) != watches_.end()) {
    watches_.emplace(name, StopWatch());
  }
  return watches_[name];
}
void StopWatchManger::PrintAll() {
  LOG(INFO) << "====="
            << "StopWatch " << name_ << "=====";
  for (auto it = watches_.begin(); it != watches_.end(); it++) {
    LOG(INFO) << std::setw(25) << std::setfill(' ') << it->first
              << "   total:" << it->second.summed_duration_ << "\t avg: "
              << it->second.summed_duration_ /
                     double(it->second.num_observations_)
              << std::endl;
  }
}

void StopWatchManger::PrintAllEveryN(size_t n) {
  ++request_since_last_print_;
  if (request_since_last_print_ == n || n == 0) {
    request_since_last_print_ = 0;
    PrintAll();
  }
}

StopWatchManger::~StopWatchManger() { PrintAll(); }

}  // namespace metrics
}  // namespace cartographer