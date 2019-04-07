/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/2d/edf_2d.h"

#include <queue>
#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

EDF2D::EDF2D(const MapLimits& limits, float truncation_distance,
               ValueConversionTables* conversion_tables)
    : Grid2D(limits, 0, truncation_distance,
             conversion_tables),
      conversion_tables_(conversion_tables),
      value_converter_(absl::make_unique<TSDValueConverter>(
          truncation_distance, 0., conversion_tables_)) {
}


bool EDF2D::CellIsUpdated(const Eigen::Array2i& cell_index) const {
  const int flat_index = ToFlatIndex(cell_index);
  uint16 tsdf_cell = correspondence_cost_cells()[flat_index];
  return tsdf_cell >= value_converter_->getUpdateMarker();
}

void EDF2D::SetCell(const Eigen::Array2i& cell_index, float tsd) {
  const int flat_index = ToFlatIndex(cell_index);
  uint16* tsdf_cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*tsdf_cell >= value_converter_->getUpdateMarker()) {
    // return;
  } else {
    mutable_update_indices()->push_back(flat_index);
    mutable_known_cells_box()->extend(cell_index.matrix());
  }
  *tsdf_cell =
      value_converter_->TSDToValue(tsd) + value_converter_->getUpdateMarker();
}

GridType EDF2D::GetGridType() const { return GridType::TSDF; }

float EDF2D::GetTSD(const Eigen::Array2i& cell_index) const {
  if (limits().Contains(cell_index)) {
    return value_converter_->ValueToTSD(
        correspondence_cost_cells()[ToFlatIndex(cell_index)]);
  }
  return value_converter_->getMaxTSD();
}

float EDF2D::GetWeight(const Eigen::Array2i& cell_index) const {
  return 0.f;
}

std::pair<float, float> EDF2D::GetTSDAndWeight(
    const Eigen::Array2i& cell_index) const {
  if (limits().Contains(cell_index)) {
    int flat_index = ToFlatIndex(cell_index);
    return std::make_pair(
        value_converter_->ValueToTSD(correspondence_cost_cells()[flat_index]),
        0.f);
  }
  return std::make_pair(value_converter_->getMaxTSD(),
                        0.f);
}

void EDF2D::GrowLimits(const Eigen::Vector2f& point) {
  Grid2D::GrowLimits(point,
                     {mutable_correspondence_cost_cells()},
                     {value_converter_->getUnknownTSDValue()});
}

proto::Grid2D EDF2D::ToProto() const {
  proto::Grid2D result;
  result = Grid2D::ToProto();
  result.mutable_tsdf_2d()->set_truncation_distance(
      value_converter_->getMaxTSD());
  return result;
}

std::unique_ptr<Grid2D> EDF2D::ComputeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<EDF2D> cropped_grid = absl::make_unique<EDF2D>(
      MapLimits(resolution, max, cell_limits), value_converter_->getMaxTSD(), conversion_tables_);
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) continue;
    cropped_grid->SetCell(xy_index, GetTSD(xy_index + offset));
  }
  cropped_grid->FinishUpdate();
  return std::move(cropped_grid);
}

bool EDF2D::DrawToSubmapTexture(
    proto::SubmapQuery::Response::SubmapTexture* const texture,
    transform::Rigid3d local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset) || GetTSD(xy_index + offset) < -0.00) {
      cells.push_back(0);  // value
      cells.push_back(0);  // alpha
      continue;
    }
    // We would like to add 'delta' but this is not possible using a value and
    // alpha. We use premultiplied alpha, so when 'delta' is positive we can
    // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
    // zero, and use 'alpha' to subtract. This is only correct when the pixel
    // is currently white, so walls will look too gray. This should be hard to
    // detect visually for the user, though.
    float normalized_tsdf = std::abs(GetTSD(xy_index + offset));
    normalized_tsdf =
        std::pow(normalized_tsdf / value_converter_->getMaxTSD(), 3.7f);
    const int delta = static_cast<int>(
        std::round((normalized_tsdf * 255. - 128.)));
    const uint8 alpha = delta > 0 ? 0 : -delta;
    const uint8 value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  common::FastGzipString(cells, texture->mutable_cells());
  texture->set_width(cell_limits.num_x_cells);
  texture->set_height(cell_limits.num_y_cells);
  const double resolution = limits().resolution();
  texture->set_resolution(resolution);
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  *texture->mutable_slice_pose() = transform::ToProto(
      local_pose.inverse() *
      transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.)));

  return true;
}

EDF2D CreateEDFFromTSDF(float truncation_distance,
                          ValueConversionTables* conversion_tables,
                          const TSDF2D& tsdf) {
  EDF2D edf(tsdf.limits(), truncation_distance, conversion_tables);
  std::queue<Eigen::Array2i> update_queue;

  // Find seeds
  const cartographer::mapping::MapLimits& limits = tsdf.limits();
  int num_x_cells = limits.cell_limits().num_y_cells;
  int num_y_cells = limits.cell_limits().num_x_cells;
  for (int ix = 0; ix < num_x_cells; ++ix) {
    for (int iy = 0; iy < num_y_cells; ++iy) {
      if (std::abs(tsdf.GetTSD({iy, ix})) <= tsdf.limits().resolution()) {
        update_queue.push({iy, ix});
        edf.SetCell({iy, ix}, std::abs(tsdf.GetTSD({iy, ix})));
      }
    }
  }

  const float discretization_threshold = 0.01 * edf.limits().resolution();
  while (!update_queue.empty()) {
    Eigen::Array2i cell_idx = update_queue.front();
    update_queue.pop();
    float center_tsd = edf.GetTSD(cell_idx);
    for (int ix = -1; ix < 2; ix++) {
      for (int iy = -1; iy < 2; iy++) {
        if (ix == 0 && iy == 0) continue;
        Eigen::Array2i candidate = {cell_idx[0] + iy, cell_idx[1] + ix};
        if (!edf.limits().Contains(candidate)) continue;
        float candidate_tsd = edf.GetTSD(candidate);
        float d =
            std::sqrt(std::abs(ix) + std::abs(iy)) * edf.limits().resolution();

        if (center_tsd + d <
            std::abs(candidate_tsd) - discretization_threshold) {
          edf.SetCell(candidate, center_tsd + d);
          update_queue.push(candidate);
        }
      }
    }
  }
  return edf;
}

}  // namespace mapping
}  // namespace cartographer
