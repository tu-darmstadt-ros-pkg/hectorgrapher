
#include "cartographer/evaluation/grid_drawer.h"
#include <cartographer/mapping/internal/2d/scan_matching/fast_esdf_scan_matcher_2d.h>
#include "cartographer/evaluation/marching_squares.h"
#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"

namespace cartographer {
namespace evaluation {
namespace {

struct RangeDataSorter {
  RangeDataSorter(Eigen::Vector3f origin) { origin_ = origin.head<2>(); }
  bool operator()(const sensor::RangefinderPoint& lhs,
                  const sensor::RangefinderPoint& rhs) {
    const Eigen::Vector2f delta_lhs =
        (lhs.position.head<2>() - origin_).normalized();
    const Eigen::Vector2f delta_rhs =
        (rhs.position.head<2>() - origin_).normalized();
    if ((delta_lhs[1] < 0.f) != (delta_rhs[1] < 0.f)) {
      return delta_lhs[1] < 0.f;
    } else if (delta_lhs[1] < 0.f) {
      return delta_lhs[0] < delta_rhs[0];
    } else {
      return delta_lhs[0] > delta_rhs[0];
    }
  }

 private:
  Eigen::Vector2f origin_;
};
}  // namespace

GridDrawer::GridDrawer(const cartographer::mapping::MapLimits& limits)
    : limits_(limits) {
  double scale = 1. / limits.resolution();
  int scaled_num_x_cells = limits.cell_limits().num_y_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_x_cells * scale;
  grid_surface_ = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context_ = cairo_create(grid_surface_);
  cairo_device_to_user_distance(grid_surface_context_, &scale, &scale);
  cairo_set_source_rgba(grid_surface_context_, 1, 1, 1, 1);
  cairo_paint(grid_surface_context_);
}

void GridDrawer::DrawTSD(const cartographer::mapping::TSDF2D& grid) {
  double scale = 1. / limits_.resolution();
  int scaled_num_x_cells = limits_.cell_limits().num_y_cells * scale;
  int scaled_num_y_cells = limits_.cell_limits().num_x_cells * scale;
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float val = grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
      if (val > 0.f) {
        g = 1. - std::pow(std::abs(val), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(val), 0.5);
        g = r;
      }
      r = 0.2 + 0.6* std::abs(val);
      g = 0.2 + 0.6* std::abs(val);
      b = 0.2 + 0.6* std::abs(val);
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawED(const cartographer::mapping::EDF2D& grid) {
  double scale = 1. / limits_.resolution();
  int scaled_num_x_cells = limits_.cell_limits().num_y_cells * scale;
  int scaled_num_y_cells = limits_.cell_limits().num_x_cells * scale;
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float val = grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
      if (val > 0.f) {
        g = 1. - std::pow(std::abs(val), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(val), 0.5);
        g = r;
      }
      r = 0.2 + 0.6 * std::abs(val);
      g = 0.2 + 0.6 * std::abs(val);
      b = 0.2 + 0.6 * std::abs(val);
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawWeights(const cartographer::mapping::TSDF2D& grid) {
  double scale = 1. / limits_.resolution();
  int scaled_num_x_cells = limits_.cell_limits().num_y_cells * scale;
  int scaled_num_y_cells = limits_.cell_limits().num_x_cells * scale;
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float val =
          grid.GetWeight({iy, ix}) / grid.value_converter_->getMaxWeight();
      if (val > 0.f) {
        g = 1. - std::pow(std::abs(val), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(val), 0.5);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context_, r, g, b);
      cairo_rectangle(grid_surface_context_, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context_);
    }
  }
}

void GridDrawer::DrawScan(
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(matched_transform.cast<float>()));

  double scale = 1. / limits_.resolution();

  // Scan Points
  cairo_set_source_rgb(grid_surface_context_, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context_, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context_);

  cairo_set_source_rgb(grid_surface_context_, 0.0, 0.8, 0);
  for (auto& scan : matched_range_data.returns) {
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context_, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context_);
}

void GridDrawer::DrawPointcloud(
    const sensor::PointCloud& range_data,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform) {
  sensor::PointCloud initial_pose_estimate_range_data =
      cartographer::sensor::TransformPointCloud(
          range_data, transform::Embed3D(initial_transform.cast<float>()));
  sensor::PointCloud matched_range_data =
      cartographer::sensor::TransformPointCloud(
          range_data, transform::Embed3D(matched_transform.cast<float>()));

  double scale = 1. / limits_.resolution();

  // Scan Points
//  cairo_set_source_rgb(grid_surface_context_, 0.8, 0.0, 0);
//  for (auto& scan : initial_pose_estimate_range_data) {
//    float x = scale * (limits_.max().x() - scan.position[0]);
//    float y = scale * (limits_.max().y() - scan.position[1]);
//    cairo_rectangle(grid_surface_context_, (x - 0.25) * scale,
//                    (y - 0.25) * scale, 0.5 * scale, 0.5 * scale);
//  }
//  cairo_fill(grid_surface_context_);

  cairo_set_source_rgb(grid_surface_context_, 0.0, 0.0, 0);
  for (auto& scan : matched_range_data) {
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context_, (x - 0.125) * scale,
                    (y - 0.125) * scale, 0.25 * scale, 0.25 * scale);
  }
  cairo_fill(grid_surface_context_);
}

void GridDrawer::DrawScanNormals(
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  sensor::RangeData transformed_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(transform.cast<float>()));
  double scale = 1. / limits_.resolution();

  // Scan Points

  // Scan Normals
  sensor::RangeData sorted_range_data = transformed_range_data;
  std::vector<std::pair<float, float>> normals;
  std::sort(sorted_range_data.returns.begin(), sorted_range_data.returns.end(),
            RangeDataSorter(sorted_range_data.origin));
  normals = cartographer::mapping::EstimateNormals(
      sorted_range_data, options.tsdf_range_data_inserter_options_2d()
                             .normal_estimation_options());
  cairo_set_source_rgb(grid_surface_context_, 0.3, 0.8, 0);
  cairo_set_line_width(grid_surface_context_, 1);
  int return_idx = 0;
  for (auto& scan : sorted_range_data.returns) {
    float cr = return_idx % 2 == 0 ? 0.8 : 0.2;
    cr = (10.f * float(return_idx) / float(sorted_range_data.returns.size()));
    cr -= floor(cr);
    cr = 0.8;
    cairo_set_source_rgb(grid_surface_context_, 1. - cr, cr, 0);
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    float dx = -1. * cos(normals[return_idx].first);
    float dy = -1. * sin(normals[return_idx].first);
    cairo_move_to(grid_surface_context_, x * scale, y * scale);
    cairo_line_to(grid_surface_context_, (x + 3*dx) * scale, (y + 3*dy) * scale);
    return_idx++;
    cairo_stroke(grid_surface_context_);
  }
}

void GridDrawer::DrawTSDFNormals(
    const cartographer::mapping::TSDF2D& grid,
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& transform) {
  sensor::RangeData transformed_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(transform.cast<float>()));

  double scale = 1. / limits_.resolution();

  // Normals from Map
  std::vector<std::pair<float, float>> normals =
      cartographer::mapping::EstimateNormalsFromTSDF(transformed_range_data,
                                                     grid);
  cairo_set_source_rgb(grid_surface_context_, 0.3, 0.3, 0.3);
  cairo_set_line_width(grid_surface_context_, 1);
  int return_idx = 0;
  for (auto& scan : transformed_range_data.returns) {
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    float dx = -1. * cos(normals[return_idx].first);
    float dy = -1. * sin(normals[return_idx].first);
    cairo_move_to(grid_surface_context_, x * scale, y * scale);
    cairo_line_to(grid_surface_context_, (x + dx) * scale, (y + dy) * scale);
    return_idx++;
    cairo_stroke(grid_surface_context_);
  }
}



void GridDrawer::DrawWeightedTSDFNormals(
    const cartographer::mapping::TSDF2D& grid,
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& transform, float max_weight) {
  sensor::RangeData transformed_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(transform.cast<float>()));
  double scale = 1. / limits_.resolution();
  // Normals from Map
  std::vector<std::pair<float, float>> normals =
      cartographer::mapping::EstimateNormalsFromTSDF(transformed_range_data,
                                                     grid);
  DrawWeightedNormals(normals, range_data, transform, max_weight);
}

void GridDrawer::DrawWeightedScanNormals(
    const sensor::RangeData& range_data,
    const cartographer::transform::Rigid2d& transform,
const cartographer::mapping::proto::TSDFRangeDataInserterOptions2D& options,
float max_weight) {
  sensor::RangeData transformed_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(transform.cast<float>()));
  double scale = 1. / limits_.resolution();

  // Scan Normals
  sensor::RangeData sorted_range_data = transformed_range_data;
  std::vector<std::pair<float, float>> normals;
  std::sort(sorted_range_data.returns.begin(), sorted_range_data.returns.end(),
            RangeDataSorter(sorted_range_data.origin));
  normals = cartographer::mapping::EstimateNormals(
      sorted_range_data, options.normal_estimation_options());

  DrawWeightedNormals(normals, range_data, transform, max_weight);
}


void GridDrawer::DrawWeightedNormals(std::vector<std::pair<float, float>> normals,
                         const sensor::RangeData& range_data,
                         const cartographer::transform::Rigid2d& transform, float max_weight) {
  double scale = 1. / limits_.resolution();
  sensor::RangeData transformed_range_data =
      cartographer::sensor::TransformRangeData(
          range_data, transform::Embed3D(transform.cast<float>()));

  cairo_set_line_width(grid_surface_context_, 1);
  int return_idx = 0;
  for (auto& scan : transformed_range_data.returns) {
    float g = std::min(2.f - 2.f * normals[return_idx].second/max_weight, 1.f);
    float r = std::min(2.f * normals[return_idx].second/max_weight, 1.f);
    cairo_set_source_rgb(grid_surface_context_, r, g, 0.7);
    float x = scale * (limits_.max().x() - scan.position[0]);
    float y = scale * (limits_.max().y() - scan.position[1]);
    float dx = -3. * cos(normals[return_idx].first);
    float dy = -3. * sin(normals[return_idx].first);
    cairo_move_to(grid_surface_context_, x * scale, y * scale);
    cairo_line_to(grid_surface_context_, (x + dx) * scale, (y + dy) * scale);
    cairo_stroke(grid_surface_context_);
    return_idx++;
  }
}

void GridDrawer::DrawIsoSurface(const cartographer::mapping::TSDF2D& grid) {
  double scale = 1. / limits_.resolution();
  // IsoSurface
  std::vector<std::vector<Eigen::Vector2f>> surface = computeSurfaces(grid);
  for (auto& segment : surface) {
    cairo_set_source_rgb(grid_surface_context_, 0, 0, 0);
    float x = scale * (limits_.max().x() - segment[0][0]);
    float y = scale * (limits_.max().y() - segment[0][1]);
    float x2 = scale * (limits_.max().x() - segment[1][0]);
    float y2 = scale * (limits_.max().y() - segment[1][1]);
    cairo_move_to(grid_surface_context_, x * scale, y * scale);
    cairo_line_to(grid_surface_context_, x2 * scale, y2 * scale);
    cairo_stroke(grid_surface_context_);
  }
}

void GridDrawer::DrawBBBounds(
    const std::vector<
        cartographer::mapping::scan_matching::BBEvaluatedCandidates>
        candidates,
    const transform::Rigid2d& initial_pose_estimate) {
  double scale = 1. / limits_.resolution();

  cairo_set_source_rgb(grid_surface_context_, 1, 0, 0);
  float x =
      scale * (limits_.max().x() - initial_pose_estimate.translation().x());
  float y =
      scale * (limits_.max().y() - initial_pose_estimate.translation().y());
  cairo_arc(grid_surface_context_, x * scale, y * scale, (0.05) * scale * scale,
            0, 2.0 * M_PI);
  cairo_fill(grid_surface_context_);

  cairo_set_source_rgb(grid_surface_context_, 0, 1, 0);
  cairo_set_line_width(grid_surface_context_, 0.5);
  for (const auto& c : candidates) {
    //    if (c.score < 0.13) {
    //      cairo_set_source_rgb(grid_surface_context_, 1, 1. - c.score / 0.13,
    //      1);
    //    } else {
    //      cairo_set_source_rgb(grid_surface_context_, 0, 0, 0);
    //    }
    float x = scale * (limits_.max().x() - c.x -
                       initial_pose_estimate.translation().x());
    float y = scale * (limits_.max().y() - c.y -
                       initial_pose_estimate.translation().y());
    cairo_arc(grid_surface_context_, x * scale, y * scale,
              (c.search_bound + 0.01) * scale * scale, 0, 2.0 * M_PI);
    cairo_stroke(grid_surface_context_);
  }
}

void GridDrawer::ToFile(std::string filename) {
  cairo_surface_write_to_png(grid_surface_, filename.c_str());
}

}  // namespace evaluation
}  // namespace cartographer