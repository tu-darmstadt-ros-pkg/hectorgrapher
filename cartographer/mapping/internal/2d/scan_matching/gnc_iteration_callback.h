#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GNC_ITERATION_CALLBACK_H
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GNC_ITERATION_CALLBACK_H

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

class GncIterationCallback : public ceres::IterationCallback {
public:
  const int MAX_ITERATIONS = 80;
  const float NON_CONVEXITY_STOP = 1.0;
  const float GM_GNC_SHAPE = 9.35;
  const float MIN_CONVEXITY = 80.0;
  const float std_non_convexity_inc_factor = 1.04;
  int scan_counter = 0;

  GncIterationCallback() :
    max_iterations(MAX_ITERATIONS),
    non_convexity_stop(NON_CONVEXITY_STOP),
    gm_gnc_shape(GM_GNC_SHAPE),
    non_convexity(MIN_CONVEXITY),
    min_convexity(MIN_CONVEXITY),
    non_convexity_inc_factor(std_non_convexity_inc_factor),
    iteration_counter(0),
    max_residual(0),
    function_tolerance(1e-6),
    gradient_tolerance(1e-10) {}

  explicit GncIterationCallback(
      const cartographer::mapping::proto::GncOptions2D &options,
      const float function_tolerance = 1e-6,
      const float gradient_tolerance = 1e-10)
      : max_iterations(options.max_iterations()),
        non_convexity_stop(options.non_convexity_stop()),
        gm_gnc_shape(options.gm_shape()),
        non_convexity(options.min_convexity()),
        min_convexity(options.min_convexity()),
        non_convexity_inc_factor(options.non_convexity_inc_factor()),
        iteration_counter(0),
        options_(options),
        max_residual(0),
        function_tolerance(function_tolerance),
        gradient_tolerance(gradient_tolerance) {
//    time_t seconds;
//    time(&seconds);
//    const char * home_c_str = getenv ("HOME");
//    std::string home;
//    if (home_c_str == NULL) {
//      home = "/home/mo/";
//    } else {
//      home = home_c_str;
//      if (home.back() != '/') home += "/";
//    }
//    log_file_path = home + "GncScanMatchingLog/" + "gnc_callback_log_"
//        + std::to_string(seconds) + ".csv";
//    LOG(INFO) << "Trajectory Builder Log Path: " << log_file_path;
//    log_file.open(log_file_path);
//    log_file << "iteration,index,weight,residual,non_convexity,f\n";
//    LOG(INFO) << "INIT Parameter: " << max_iterations << ", "
//              << non_convexity_stop << ", " << gm_gnc_shape << ", "
//              << non_convexity << ", " << non_convexity_inc_factor
//              << ", " << options.max_retries();
  }

  ~GncIterationCallback() {}

  ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary) {
//    LOG(INFO) << "Callback #nr executed: " << iteration_counter << std::endl;
//    LOG(INFO) << "Cost in iteration #" << iteration_counter << " Cost: " << summary.cost << "; Change: " << summary.cost_change << "; MaxNorm" << summary.gradient_max_norm;
//    if (summary.iteration == 0) LOG(INFO) << "1. Score: " << summary.cost << ", d: " << summary.cost_change;

    iteration_counter = summary.iteration + 1;  // + 1 for set_max_distance(...)
//    if (iteration_counter > 8) {
    float f = non_convexity * pow(gm_gnc_shape, 2);
    for (size_t i = 0; i < gnc_weights.size(); i++) {
      gnc_weights[i] = pow((f / (f + pow(gnc_distances[i], 1))), 2);
//      log_file << summary.iteration << "," << i << "," << gnc_weights[i] << ","
//               << gnc_distances[i] << "," << non_convexity << "," << f << "\n";
//      if (i == 0) LOG(INFO) << "WEIGHT 0: " << gnc_weights[i] << "D: " << gnc_distances[i];
//      if (gnc_weights[i] < 0.5) gnc_weights[i] = 0.5;
//      LOG(INFO) << "DIST: " << i << "; " << gnc_distances[i] << " Weight: "
//                << gnc_weights[i];
    }
//    }

//    double a = (non_convexity + 1) / non_convexity;
//    double b = 1 / a;
//    double shape = pow(gm_gnc_shape, 2);
//    LOG(INFO) << "A:" << a * shape << "B: " << b * shape;
//    for (size_t i = 0; i < gnc_weights.size(); i++) {
//      if (gnc_distances[i] > a * shape)
//        gnc_weights[i] = 0;
//      if (gnc_distances[i] > b * shape && gnc_distances[i] <= a * shape)
//        gnc_weights[i] = gm_gnc_shape / sqrt(gnc_distances[i]) *
//                         sqrt(non_convexity * (non_convexity + 1)) -
//                         non_convexity;
//      if (gnc_distances[i] <= b * shape)
//        gnc_weights[i] = 1;

//    }

//    assert(summary.iteration == iteration_counter);

    if (summary.iteration > 1 && summary.step_is_successful && // cost change only after first iteration available
        (summary.iteration > max_iterations ) ||
        non_convexity <= non_convexity_stop
//        summary.cost_change < function_tolerance * 1e-4 ||
//        summary.gradient_norm < gradient_tolerance)
        ) {  // TODO > or < depending on cost fct
//      LOG(INFO) << "Non Convexity unsuccessful for match: #" << scan_counter
//                << " after iteration: " << iteration_counter
//                << " and end convexity: " << non_convexity;
//      return ceres::SOLVER_AB1ORT;
//      LOG(INFO) << "FUNCTION TOLERANCE: " << function_tolerance << ", " << summary.cost_change;
//      LOG(INFO) << "GRADIENT TOLERANCE: " << gradient_tolerance << ", " << summary.gradient_norm;
//      LOG(INFO) << "END NON CONVEXITY: " << non_convexity;
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    } else {
      if (summary.step_is_successful)  // TODO
        non_convexity_increment();
//      LOG(INFO) << "SUM: " << iteration_counter << ":" <<
//        std::accumulate(gnc_weights.begin(), gnc_weights.end(), 0.0);
//      iteration_counter++;
      return ceres::SOLVER_CONTINUE;
    }
  }

  void non_convexity_increment() {
    // TLS
//    non_convexity = non_convexity * non_convexity_inc_factor;
    // GM
    non_convexity = non_convexity / non_convexity_inc_factor;
    if (non_convexity < 1) non_convexity = 1;  // TODO
//    if (non_convexity < non_convexity_stop) non_convexity = non_convexity_stop;  // TODO
  }

  void newMatch(const unsigned long new_size) {
    ++scan_counter;
    iteration_counter = 0;
//    non_convexity = MIN_CONVEXITY;
//    non_convexity_inc_factor = std_non_convexity_inc_factor;
    gnc_weights.clear();
    gnc_distances.clear();
    gnc_weights.resize(new_size);
    gnc_distances.resize(new_size);
    std::fill(gnc_weights.begin(), gnc_weights.end(), 1);
  };

  void newMatch(const unsigned long new_size, const float start_convexity) {
    iteration_counter = 0;
//    non_convexity = non_convexity * 2;
//    non_convexity = sqrt(sqrt(non_convexity));
    gm_gnc_shape /= 1.4;
//    non_convexity_inc_factor = start_convexity;
    non_convexity_inc_factor = (sqrt(non_convexity_inc_factor));
    gnc_weights.clear();
    gnc_distances.clear();
    gnc_weights.resize(new_size);
    gnc_distances.resize(new_size);
    std::fill(gnc_weights.begin(), gnc_weights.end(), 1);
  };

  std::vector<double> *get_weights() const {
    return &gnc_weights;
  }
  void set_weights(std::vector<double> &weights) const {
    this->gnc_weights = weights;
  }

  std::vector<double> *get_distances() {
    return &gnc_distances;
  }

  const float get_non_convexity() const {
    return non_convexity;
  }

  const float get_gm_gnc_shape() const {
    return gm_gnc_shape;
  }

  float get_non_convexity_inc_factor() const {
    return non_convexity_inc_factor;
  }

  void set_max_residual(float residual) const {
    if (this->iteration_counter == 0) {
      max_residual = residual;

      // TLS
//      non_convexity = pow(gm_gnc_shape, 2) /
//                      (2 * pow(max_residual, 1) - pow(gm_gnc_shape, 2));
//      if (non_convexity < min_convexity) non_convexity = min_convexity;

      // GM
      non_convexity = 2 * pow(max_residual, 1) * scaling_factor /  // TODO pow(.., 1 or 2)
                      pow(gm_gnc_shape, 2);
//      LOG(INFO) << "MAX Residual: "  << max_residual << ", Conv.: " << non_convexity;
      if (non_convexity < min_convexity) non_convexity = min_convexity;
//      LOG(INFO) << "GM-Gnc Start Convexity: " << non_convexity;
//      LOG(INFO) << "Increment factor: " << non_convexity_inc_factor << " SHAPE: " << gm_gnc_shape;
    }
  }

  void set_distance(size_t pos, double value) const {
//    LOG(INFO) << "Size:" << gnc_distances.size() << " Pos:" << pos;
//    LOG(INFO) << "Dist: " << value << " Pos: " << pos;
    gnc_distances.at(pos) = value * scaling_factor;
  }

//  void set_ceres_solver_options(ceres::Solver::Options *options) {
//    this->ceres_solver_options = options;
//  }

private:
//  int iteration_counter;
  int max_iterations;
  float non_convexity_stop;
  float gm_gnc_shape;
  mutable float non_convexity;  // mutable, to change in variable update
  float min_convexity;
  float non_convexity_inc_factor;
  int iteration_counter;
  const cartographer::mapping::proto::GncOptions2D options_;
  mutable float max_residual;
  const float function_tolerance;
  const float gradient_tolerance;
  mutable std::vector<double> gnc_weights;
  mutable std::vector<double> gnc_distances;
  const float scaling_factor = 100000.0;
  std::ofstream log_file;
  std::string log_file_path;
//  ceres::Solver::Options *ceres_solver_options;
};

} // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif //CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GNC_ITERATION_CALLBACK_H
