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

#ifndef CARTOGRAPHER_MAPPING_3D_TSDF_RANGE_DATA_INSERTER_3D_H_
#define CARTOGRAPHER_MAPPING_3D_TSDF_RANGE_DATA_INSERTER_3D_H_

#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/proto/3d/range_data_inserter_options_3d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

proto::TSDFRangeDataInserterOptions3D CreateTSDFRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

class TSDFRangeDataInserter3D : public RangeDataInserterInterface {
 public:
  explicit TSDFRangeDataInserter3D(
      const proto::TSDFRangeDataInserterOptions3D& options);

  TSDFRangeDataInserter3D(const TSDFRangeDataInserter3D&) = delete;
  TSDFRangeDataInserter3D& operator=(const TSDFRangeDataInserter3D&) = delete;

  //  // Inserts 'range_data' into 'hybrid_grid'.
  //  void Insert(const sensor::RangeData& range_data,
  //              HybridGrid* hybrid_grid) const;

  virtual void Insert(const sensor::RangeData& range_data,
                      GridInterface* grid) const override;

 private:
  void InsertHit(const Eigen::Vector3f& hit, const Eigen::Vector3f& origin,
                 HybridGridTSDF* tsdf) const;
  void InsertHitWithNormal(const Eigen::Vector3f& hit,
                           const Eigen::Vector3f& origin,
                           const Eigen::Vector3f& normal,
                           HybridGridTSDF* tsdf) const;
  void InsertTriangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                      const Eigen::Vector3f& v2, const Eigen::Vector3f& origin,
                      HybridGridTSDF* tsdf) const;
  void RasterTriangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                      const Eigen::Vector3f& v2,
                      const Eigen::Vector3f& triangle_normal, float tsd_offset,
                      HybridGridTSDF* tsdf) const;
  void UpdateCell(const Eigen::Array3i& cell, float update_sdf,
                  float update_weight, HybridGridTSDF* tsdf) const;

  virtual bool RequiresStructuredData() const override { return true; };

  const proto::TSDFRangeDataInserterOptions3D options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_TSDF_RANGE_DATA_INSERTER_3D_H_
