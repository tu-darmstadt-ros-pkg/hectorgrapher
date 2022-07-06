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

#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_DIRECTIONAL_TSDF_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_DIRECTIONAL_TSDF_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/tsd_value_converter.h"
#include "cartographer/mapping/3d/hybrid_grid_base.h"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/grid_interface.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/3d/hybrid_grid_tsdf.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
size_t NormalToIndex(const Eigen::Vector3f& normal){
  Eigen::Vector3f abs_normal = normal.cwiseAbs();
  size_t max_index = abs_normal.maxCoeff();
  if(abs_normal[max_index] < 0.) max_index += 3;
  return max_index;
//  if( abs_normal.x() > abs_normal.y() && abs_normal.x() > abs_normal.z() ) {
//    if(normal.x() >= 0) return 0;
//    else return 1;
//  }
//  else if (abs_normal.y() > abs_normal.z()) {
//    if(normal.y() >= 0) return 2;
//    else return 3;
//  }
//  else {
//    if(normal.z() >= 0) return 4;
//    else return 5;
//  }
}


class HybridGridDirectionalTSDF : public GridInterface{
 public:
  explicit HybridGridDirectionalTSDF(const float resolution,
                          float relative_truncation_distance, float max_weight,
                          ValueConversionTables* conversion_tables)
      : conversion_tables_(conversion_tables),
        value_converter_(absl::make_unique<TSDValueConverter>(
            relative_truncation_distance * resolution, max_weight,
            conversion_tables_)) {}

  explicit HybridGridDirectionalTSDF(const proto::HybridGridTSDF& proto,
                          ValueConversionTables* conversion_tables)
      : HybridGridDirectionalTSDF(proto.resolution(), proto.relative_truncation_distance(),
                       proto.max_weight(), conversion_tables) {
    LOG(FATAL) << "not implemented";
  }
  virtual GridType GetGridType() const override { return GridType::DIRECTIONAL_TSDF; };

  // Sets the probability of the cell at 'index' to the given 'probability'.
  void SetCell(const Eigen::Array3i& index, const Eigen::Vector3f normal, const float tsd,
               const float weight) {
    size_t grid_idx = NormalToIndex(normal);
    tsdf_grids_[grid_idx].SetCell(index, tsd, weight);
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    for(auto& grid : tsdf_grids_) {
      grid.FinishUpdate();
    }
  }

  // Returns the truncated signed distance of the cell with 'index'.
  float GetTSD(const Eigen::Array3i& index) const {
    float tsd = value_converter_->getMaxTSD();
    for(auto& grid : tsdf_grids_) {
      if(std::abs(grid.GetTSD(index)) < tsd) tsd = grid.GetTSD(index);
    }
    return tsd;
  }

  // Returns the weight of the cell with 'index'.
  float GetWeight(const Eigen::Array3i& index) const {
    float weight = 0.f;
    for(auto& grid : tsdf_grids_) {
      if(std::abs(grid.GetWeight(index)) > weight) weight = grid.GetWeight(index);
    }
    return weight;
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const {
    for(auto& grid : tsdf_grids_) {
      if(grid.IsKnown(index)) return true;
    }
    return false;
  }

  proto::HybridGridTSDF ToProto() const {
    LOG(FATAL) << "Not implemented";
    proto::HybridGridTSDF result;
    return result;
  }

  const TSDValueConverter& ValueConverter() const { return *value_converter_; }

 private:
  // Markers at changed cells.
  std::vector<HybridGridTSDF> tsdf_grids_;
  ValueConversionTables* conversion_tables_;
  std::unique_ptr<TSDValueConverter> value_converter_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_DIRECTIONAL_TSDF_H_
