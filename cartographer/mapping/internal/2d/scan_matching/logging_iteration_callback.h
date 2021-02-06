#ifndef CARTOGRAPHER_LOGGING_ITERATION_CALLBACK_H
#define CARTOGRAPHER_LOGGING_ITERATION_CALLBACK_H



#include <ceres/iteration_callback.h>
#include <numeric>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/2d/gnc_2d.pb.h"
#include "glog/logging.h"
#include <fstream>

namespace cartographer {
namespace mapping {
namespace scan_matching {

cartographer::mapping::proto::GncOptions2D CreateGncOptions2D(
    cartographer::common::LuaParameterDictionary *const parameter_dictionary);

class LoggingIterationCallback : public ceres::IterationCallback {
public:

  LoggingIterationCallback() {
//    time(&seconds);
    timespec_get(&ts, TIME_UTC);
    const char *home_c_str = getenv("HOME");
    std::string home;
    if (home_c_str == NULL) {
      home = "";
    } else {
      home = home_c_str;
      if (home.back() != '/') home += "/";
    }
    log_file_path = home + "IterationsLog/iteration_callback_log.csv";
//        + std::to_string(ts.tv_sec) + std::to_string(ts.tv_nsec) + ".csv";
    log_file.open(log_file_path, std::ios_base::app);
//    log_file << "pose_x,pose_y,pose_w,time_s\n";;
  }


  ~LoggingIterationCallback() {}

  ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) {
    if (abs(ceres_pose_estimate[0]) > 1e-100 && ceres_pose_estimate[0] != 0
        && abs(ceres_pose_estimate[0]) < 1e100) {
      log_file  << r << ","
                << summary.iteration << ","
                << abs(0.2 -sqrt(pow(ceres_pose_estimate[0], 2)
                        + pow(ceres_pose_estimate[1], 2))) << ","
                << abs(0.2 - abs(ceres_pose_estimate[2])) << ","
                << summary.iteration_time_in_seconds << ","
                << gnc << "\n";
    }
    return ceres::SOLVER_CONTINUE;
  }

  void set_ceres_pose_estimate_ptr(double *ceres_pose_estimate_ptr) {
    LOG(INFO) << "Setting pose prt";
    this->ceres_pose_estimate = ceres_pose_estimate_ptr;
//    log_file << " , , , , , \n";  // new line for new match in csv
  }

  void set_gnc(bool is_gnc) {
    this->gnc = is_gnc;
  }

private:
  bool gnc = false;
  double *ceres_pose_estimate;
  std::ofstream log_file;
  std::string log_file_path;
  time_t seconds;
  struct timespec ts;
  int r = rand();
};
}
}
}
#endif //CARTOGRAPHER_LOGGING_ITERATION_CALLBACK_H