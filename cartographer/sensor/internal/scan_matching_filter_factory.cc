/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cartographer/sensor/proto/scan_matching_filter_options.pb.h>
#include "cartographer/sensor/internal/scan_matching_filter_factory.h"

namespace cartographer {
namespace sensor {


proto::ScanMatchingFilterOptions CreateScanMatchingFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {

  std::map<std::string, proto::ScanMatchingFilterOptions::FilterType> filter_type_map = {
    { "voxel_filter", proto::ScanMatchingFilterOptions::VOXEL_FILTER },
    { "random_filter", proto::ScanMatchingFilterOptions::RANDOM_FILTER },
    { "max_entropy_normal_filter", proto::ScanMatchingFilterOptions::MAX_ENTROPY_NORMAL_FILTER }
  };
  proto::ScanMatchingFilterOptions::FilterType filter_type = proto::ScanMatchingFilterOptions::VOXEL_FILTER;
  proto::ScanMatchingFilterOptions options;

  std::string type = parameter_dictionary->GetString("type");

  auto iterator = filter_type_map.find(type);
  if(iterator != filter_type_map.end()){
    filter_type = iterator->second;
  }

  options.set_type(filter_type);

  options.set_voxel_filter_size(parameter_dictionary->GetDouble("voxel_filter_size"));
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}


}  // namespace sensor
}  // namespace cartographer
