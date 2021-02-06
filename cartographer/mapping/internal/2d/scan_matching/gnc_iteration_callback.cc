#include "cartographer/mapping/internal/2d/scan_matching/gnc_iteration_callback.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

cartographer::mapping::proto::GncOptions2D CreateGncOptions2D(
    cartographer::common::LuaParameterDictionary *const parameter_dictionary) {
  cartographer::mapping::proto::GncOptions2D options;
  options.set_use_gnc(
      parameter_dictionary->GetBool("use_gnc"));
  options.set_max_iterations(
      parameter_dictionary->GetInt("max_iterations"));
  options.set_non_convexity_stop(
      parameter_dictionary->GetDouble("non_convexity_stop"));
  options.set_gm_shape(parameter_dictionary->GetDouble("gm_shape"));
  options.set_min_convexity(parameter_dictionary->GetDouble("min_convexity"));
  options.set_non_convexity_inc_factor(
      parameter_dictionary->GetDouble("non_convexity_inc_factor"));
  options.set_max_retries(
      parameter_dictionary->GetInt("max_retries"));
  return options;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer