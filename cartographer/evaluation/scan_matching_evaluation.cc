#include <chrono>
#include <fstream>
#include <random>
#include <string>

#include "cairo.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/time.h"
#include "cartographer/evaluation/marching_squares.h"
#include "cartographer/evaluation/scan_cloud_generator.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/normal_estimation_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_gnc_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "sample_generator.h"
#include <chrono>

namespace cartographer {
namespace evaluation {
namespace {

template<typename Base, typename T>
inline bool instanceof(const T*) {
  return std::is_base_of<Base, T>::value;
}

static int rendered_grid_id = 0;
static mapping::ValueConversionTables conversion_tables;
static bool do_render = true;

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


template <typename GridType>
std::unique_ptr<GridType> generateGrid(
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  std::unique_ptr<GridType> grid = absl::make_unique<GridType>(
      cartographer::mapping::MapLimits(
          0.05, Eigen::Vector2d(1., 1.),
          cartographer::mapping::CellLimits(40, 40)),
      &conversion_tables);
  return std::move(grid);
}

template <>
std::unique_ptr<cartographer::mapping::TSDF2D>
generateGrid<cartographer::mapping::TSDF2D>(
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  std::unique_ptr<cartographer::mapping::TSDF2D> grid =
      absl::make_unique<cartographer::mapping::TSDF2D>(
          cartographer::mapping::MapLimits(
              0.05, Eigen::Vector2d(1., 1.),
              cartographer::mapping::CellLimits(40, 40)),
          options.tsdf_range_data_inserter_options_2d().truncation_distance(),
          options.tsdf_range_data_inserter_options_2d().maximum_weight(),
          &conversion_tables);
  return std::move(grid);
}

template <typename RangeDataInserter>
std::unique_ptr<RangeDataInserter> generateRangeDataInserter(
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options) {
  LOG(ERROR) << " Undefined RangeDataInserter Type ";
  std::unique_ptr<RangeDataInserter> inserter;
  return std::move(inserter);
}

template <>
std::unique_ptr<cartographer::mapping::TSDFRangeDataInserter2D>
generateRangeDataInserter<cartographer::mapping::TSDFRangeDataInserter2D>(
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options) {
  std::unique_ptr<cartographer::mapping::TSDFRangeDataInserter2D> inserter =
      absl::make_unique<cartographer::mapping::TSDFRangeDataInserter2D>(
          range_data_inserter_options.tsdf_range_data_inserter_options_2d());
  return std::move(inserter);
}

template <>
std::unique_ptr<cartographer::mapping::ProbabilityGridRangeDataInserter2D>
generateRangeDataInserter<
    cartographer::mapping::ProbabilityGridRangeDataInserter2D>(
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options) {
  std::unique_ptr<cartographer::mapping::ProbabilityGridRangeDataInserter2D>
      inserter = absl::make_unique<
          cartographer::mapping::ProbabilityGridRangeDataInserter2D>(
          range_data_inserter_options
              .probability_grid_range_data_inserter_options_2d());
  return std::move(inserter);
}

void renderGridwithScan(
    const cartographer::mapping::ProbabilityGrid& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options,
    const std::vector<double> *gnc_weights) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(matched_transform.cast<float>()));

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float p = 1. - grid.GetProbability({iy, ix});
      cairo_set_source_rgb(grid_surface_context, p, p, p);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
  for (auto& scan : matched_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }

  cairo_fill(grid_surface_context);

  time_t seconds;
  time(&seconds);
  std::string filename = "grid_with_inserted_cloud" + std::to_string(seconds) +
                         std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}

void renderGridWeightswithScan(
    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(matched_transform.cast<float>()));

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          grid.GetWeight({iy, ix}) /
          options.tsdf_range_data_inserter_options_2d().maximum_weight();
      if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.5);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context, r, g, b);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  // Scan Points
  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
  for (auto& scan : matched_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  // Scan Normals
  sensor::RangeData sorted_range_data = matched_range_data;
  std::vector<std::pair<float, float>> normals;

//  std::chrono::steady_clock::time_point begin =
//      std::chrono::steady_clock::now();
  std::sort(sorted_range_data.returns.begin(), sorted_range_data.returns.end(),
            RangeDataSorter(sorted_range_data.origin));
  normals = cartographer::mapping::EstimateNormals(
      sorted_range_data, options.tsdf_range_data_inserter_options_2d()
                             .normal_estimation_options());

//  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  //  std::cout << "Time difference = "
  //            << std::chrono::duration_cast<std::chrono::nanoseconds>(end -
  //            begin)
  //                   .count()
  //            << std::endl;
  cairo_set_source_rgb(grid_surface_context, 0.3, 0.8, 0);
  cairo_set_line_width(grid_surface_context, 1);
  int return_idx = 0;
  for (auto& scan : sorted_range_data.returns) {
    float cr = return_idx % 2 == 0 ? 0.8 : 0.2;
    cr = (10.f * float(return_idx) / float(sorted_range_data.returns.size()));
    cr -= floor(cr);
    cr = 0.8;
    cairo_set_source_rgb(grid_surface_context, 1. - cr, cr, 0);
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    float dx = -1. * cos(normals[return_idx].first);
    float dy = -1. * sin(normals[return_idx].first);
    cairo_move_to(grid_surface_context, x * scale, y * scale);
    cairo_line_to(grid_surface_context, (x + dx) * scale, (y + dy) * scale);
    return_idx++;
    cairo_stroke(grid_surface_context);
  }

  // Normals from Map
//  std::chrono::steady_clock::time_point begintsdf =
//      std::chrono::steady_clock::now();
  normals =
      cartographer::mapping::EstimateNormalsFromTSDF(sorted_range_data, grid);
//  std::chrono::steady_clock::time_point endtsdf =
//      std::chrono::steady_clock::now();
  //  std::cout << "Time difference = "
  //            << std::chrono::duration_cast<std::chrono::nanoseconds>(endtsdf
  //            -
  //                                                                    begintsdf)
  //                   .count()
  //            << std::endl;
  cairo_set_source_rgb(grid_surface_context, 0.3, 0.8, 0);
  cairo_set_line_width(grid_surface_context, 1);
  return_idx = 0;
  for (auto& scan : sorted_range_data.returns) {
    float cr = return_idx % 2 == 0 ? 0.8 : 0.2;
    cr = (10.f * float(return_idx) / float(sorted_range_data.returns.size()));
    cr -= floor(cr);
    cr = 0.8;
    cairo_set_source_rgb(grid_surface_context, 0, 1. - cr, cr);
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    float dx = -1. * cos(normals[return_idx].first);
    float dy = -1. * sin(normals[return_idx].first);
    cairo_move_to(grid_surface_context, x * scale, y * scale);
    cairo_line_to(grid_surface_context, (x + dx) * scale, (y + dy) * scale);
    return_idx++;
    cairo_stroke(grid_surface_context);
  }

  // IsoSurface
  std::vector<std::vector<Eigen::Vector2f>> surface = computeSurfaces(grid);
  for (auto& segment : surface) {
    cairo_set_source_rgb(grid_surface_context, 1, 1, 1);
    float x = scale * (limits.max().x() - segment[0][0]);
    float y = scale * (limits.max().y() - segment[0][1]);
    float x2 = scale * (limits.max().x() - segment[1][0]);
    float y2 = scale * (limits.max().y() - segment[1][1]);
    cairo_move_to(grid_surface_context, x * scale, y * scale);
    cairo_line_to(grid_surface_context, x2 * scale, y2 * scale);
    cairo_stroke(grid_surface_context);
  }

  time_t seconds;
  time(&seconds);
  std::string filename = "grid_weights_with_inserted_cloud" +
                         std::to_string(seconds) +
                         std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
}

double get_color_from_weight(double weight) {
  if (weight < 0.1) {
    return 0.01;
  } else if (weight < 0.3) {
    return 0.1;
  } else if (weight < 0.5) {
    return 0.3;
  } else if (weight < 0.7) {
    return 0.5;
  } else if (weight < 0.9) {
    return 0.66;
  } else if (weight < 1.0) {
    return 0.75;
  } else if (weight < 1.5) {
    return 0.8;
  } else if (weight < 2) {
    return 0.82;
  } else if (weight < 5) {
    return 0.85;
  } else if (weight < 10) {
    return 0.92;
  } else if (weight < 100) {
    return 0.95;
  }
  return 1.0;
}

void renderGridwithScanBase(
    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options,
    const std::vector<double> *gnc_weights) {
  sensor::RangeData initial_pose_estimate_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(initial_transform.cast<float>()));
  sensor::RangeData matched_range_data =
      cartographer::sensor::TransformRangeData(
          sample.range_data,
          transform::Embed3D(matched_transform.cast<float>()));

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  cairo_surface_t* grid_surface;
  cairo_t* grid_surface_context;

  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;
  grid_surface = cairo_image_surface_create(
      CAIRO_FORMAT_ARGB32, scaled_num_x_cells, scaled_num_y_cells);
  grid_surface_context = cairo_create(grid_surface);
  cairo_device_to_user_distance(grid_surface_context, &scale, &scale);
  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      float r = 1.f;
      float g = 1.f;
      float b = 1.f;
      float normalized_tsdf =
          grid.GetTSD({iy, ix}) / grid.GetMaxCorrespondenceCost();
      if (normalized_tsdf > 0.99f) {
        r = 1.f;
        g = 1.f;
        b = 1.f;
      } else if (normalized_tsdf < -0.99f) {
        r = 0.7f;
        g = 0.7f;
        b = 0.7f;
      } else if (normalized_tsdf > 0.f) {
        g = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        b = g;
      } else {
        r = 1. - std::pow(std::abs(normalized_tsdf), 0.7);
        g = r;
      }
      cairo_set_source_rgb(grid_surface_context, r, g, b);
      cairo_rectangle(grid_surface_context, scale * (float(ix)),
                      scale * ((float)iy), scale, scale);
      cairo_fill(grid_surface_context);
    }
  }

  // Scan Points
  cairo_set_source_rgb(grid_surface_context, 0.8, 0.0, 0);
  for (auto& scan : initial_pose_estimate_range_data.returns) {
    float x = scale * (limits.max().x() - scan.position[0]);
    float y = scale * (limits.max().y() - scan.position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
  }
  cairo_fill(grid_surface_context);

  cairo_set_source_rgb(grid_surface_context, 0.0, 0.8, 0);
//  for (auto& scan : matched_range_data.returns) {
  for (size_t i=0; i<matched_range_data.returns.size(); i++) {
    if (gnc_weights != nullptr) {
      double color = get_color_from_weight(gnc_weights->at(i));
      cairo_set_source_rgb(grid_surface_context, color, color, color);
//      double max_ = *std::max_element(gnc_weights->begin(), gnc_weights->end());
//      double min_ = *std::min_element(gnc_weights->begin(), gnc_weights->end());
//      cairo_set_source_rgb(grid_surface_context,
//                           (gnc_weights->at(i) - min_) / (max_ - min_),
//                           (gnc_weights->at(i) - min_) / (max_ - min_),
//                           (gnc_weights->at(i) - min_) / (max_ - min_));
//      LOG(INFO) << i << ", weight: " << gnc_weights->at(i);
    }
//    LOG(INFO) << std::acc
    float x = scale *
        (limits.max().x() - matched_range_data.returns[i].position[0]);
    float y = scale *
        (limits.max().y() - matched_range_data.returns[i].position[1]);
    cairo_rectangle(grid_surface_context, (x - 0.15) * scale,
                    (y - 0.15) * scale, 0.3 * scale, 0.3 * scale);
    cairo_fill(grid_surface_context);
  }
  cairo_fill(grid_surface_context);
  //
  //    // Scan Normals
  //    sensor::RangeData sorted_range_data = matched_range_data;
  //    std::vector<float> normals;
  //    std::sort(sorted_range_data.returns.begin(),
  //    sorted_range_data.returns.end(),
  //              RangeDataSorter(sorted_range_data.origin));
  //    normals = cartographer::mapping::EstimateNormals(
  //        sorted_range_data, options.tsdf_range_data_inserter_options_2d()
  //                               .normal_estimation_options());
  //    cairo_set_source_rgb(grid_surface_context, 0.3, 0.8, 0);
  //    cairo_set_line_width(grid_surface_context, 1);
  //    int return_idx = 0;
  //    for (auto& scan : sorted_range_data.returns) {
  //      float cr = return_idx % 2 == 0 ? 0.8 : 0.2;
  //      cr = (10.f * float(return_idx) /
  //      float(sorted_range_data.returns.size()));
  //      cr -= floor(cr);
  //      cr = 0.8;
  //      cairo_set_source_rgb(grid_surface_context, 1. - cr, cr, 0);
  //      float x = scale * (limits.max().x() - scan.position[0]);
  //      float y = scale * (limits.max().y() - scan.position[1]);
  //      float dx = -1. * cos(normals[return_idx]);
  //      float dy = -1. * sin(normals[return_idx]);
  //      cairo_move_to(grid_surface_context, x * scale, y * scale);
  //      cairo_line_to(grid_surface_context, (x + dx) * scale, (y + dy) *
  //      scale);
  //      return_idx++;
  //      cairo_stroke(grid_surface_context);
  //    }
  //
  //    // Normals from Map
  //    normals =
  //        cartographer::mapping::EstimateNormalsFromTSDF(sorted_range_data,
  //        grid);
  //    cairo_set_source_rgb(grid_surface_context, 0.3, 0.3, 0.3);
  //    cairo_set_line_width(grid_surface_context, 1);
  //    return_idx = 0;
  //    for (auto& scan : sorted_range_data.returns) {
  //      float x = scale * (limits.max().x() - scan.position[0]);
  //      float y = scale * (limits.max().y() - scan.position[1]);
  //      float dx = -1. * cos(normals[return_idx]);
  //      float dy = -1. * sin(normals[return_idx]);
  //      cairo_move_to(grid_surface_context, x * scale, y * scale);
  //      cairo_line_to(grid_surface_context, (x + dx) * scale, (y + dy) *
  //      scale);
  //      return_idx++;
  //      cairo_stroke(grid_surface_context);
  //    }

  // IsoSurface
  //  std::vector<std::vector<Eigen::Vector2f>> surface = computeSurfaces(grid);
  //  for (auto& segment : surface) {
  //    cairo_set_source_rgb(grid_surface_context, 0, 0, 0);
  //    float x = scale * (limits.max().x() - segment[0][0]);
  //    float y = scale * (limits.max().y() - segment[0][1]);
  //    float x2 = scale * (limits.max().x() - segment[1][0]);
  //    float y2 = scale * (limits.max().y() - segment[1][1]);
  //    cairo_move_to(grid_surface_context, x * scale, y * scale);
  //    cairo_line_to(grid_surface_context, x2 * scale, y2 * scale);
  //    cairo_stroke(grid_surface_context);
  //  }

  time_t seconds;
  time(&seconds);
  std::string filename = "grid_with_inserted_cloud" + std::to_string(seconds) +
                         std::to_string(rendered_grid_id) + ".png";
  rendered_grid_id++;
  cairo_surface_write_to_png(grid_surface, filename.c_str());
  //    renderGridWeightswithScan(grid, sample, initial_transform,
  //    matched_transform,
  //                              options);
}

void renderGridwithScan(
    const cartographer::mapping::TSDF2D& grid, const Sample& sample,
    const cartographer::transform::Rigid2d& initial_transform,
    const cartographer::transform::Rigid2d& matched_transform,
    const cartographer::mapping::proto::RangeDataInserterOptions& options,
    const std::vector<double> *gnc_weights) {
  renderGridwithScanBase(grid, sample, initial_transform, matched_transform,
                         options, gnc_weights);
  //  cartographer::mapping::TSDF2D esdf =
  //      cartographer::mapping::CreateESDFFromTSDF(2.0f,
  //      grid.conversion_tables_,
  //                                                grid);
  //  renderGridwithScanBase(esdf, sample, initial_transform, matched_transform,
  //                         options);
}


template <typename GridType, typename ScanMatcher>
void EvaluateCostFunction(
    const Sample& sample, const GridType& grid,
    const cartographer::mapping::proto::RangeDataInserterOptions&
    range_data_inserter_options,
    const cartographer::mapping::scan_matching::proto::
    CeresScanMatcherOptions2D& ceres_scan_matcher_options,
    std::vector<std::vector<double>>* gradients) {
  float min_trans = -1.2;
  float max_trans = 1.2;
  float resolution_trans = 0.05;
  float min_rot = -0.0f;
  float max_rot = 0.00001;  // 1E-7f;  // 2.f * M_PI;
  float resolution_rot = 0.1;
  for (float x = min_trans; x <= max_trans; x += resolution_trans) {
    for (float y = min_trans; y <= max_trans; y += resolution_trans) {
      for (float theta = min_rot; theta <= max_rot; theta += resolution_rot) {
        ScanMatcher scan_matcher(ceres_scan_matcher_options);
        const Eigen::Vector2d target_translation = {0., 0.};
        const cartographer::transform::Rigid2d initial_pose_estimate =
            cartographer::transform::Rigid2d({x, y}, theta);
        cartographer::transform::Rigid2d matched_pose_estimate;
        ceres::Solver::Summary summary;
        double cost;
        std::vector<double> jacobians;

        scan_matcher.Evaluate(target_translation, initial_pose_estimate,
                              sample.range_data.returns, grid, &cost, NULL,
                              &jacobians);

        std::vector<double> result = {x, y, theta, cost};
        result.insert(result.end(), jacobians.begin(), jacobians.end());
        gradients->push_back(result);
      }
    }
//    LOG(INFO) << 100.0 * (1.0 - (max_trans - x) / (max_trans - min_trans))
//              << "%";
  }
}


template <typename GridType, typename ScanMatcher>
void MatchScan(
    Sample& sample,
    const cartographer::mapping::scan_matching::proto::
    CeresScanMatcherOptions2D& ceres_scan_matcher_options,
    const cartographer::mapping::proto::RangeDataInserterOptions& options,
    const GridType& grid, SampleResult* sample_result) {
  ScanMatcher scan_matcher(ceres_scan_matcher_options);

  const Eigen::Vector2d target_translation = {0., 0.};
  const cartographer::transform::Rigid2d initial_pose_estimate =
      cartographer::transform::Rigid2d::Translation({0.0, 0.0});
  cartographer::transform::Rigid2d matched_pose_estimate;
  ceres::Solver::Summary summary;

  auto match_start = std::chrono::high_resolution_clock::now();

  scan_matcher.Match(target_translation, initial_pose_estimate,
                     sample.range_data.returns, grid, &matched_pose_estimate,
                     &summary);

//  if (instanceof<cartographer::mapping::scan_matching::CeresScanMatcherGnc2D>
//      (&scan_matcher)) {
//    ceres::Solver::Summary summary2;
//    auto *c = dynamic_cast<cartographer::mapping::scan_matching::CeresScanMatcherGnc2D *>(&scan_matcher);
//    sensor::PointCloud gnc_point_cloud;
//    auto weights = *c->get_gnc_callback()->get_weights();
//    std::vector<double> weights_sorted(weights.size());
//    partial_sort_copy(begin(weights), end(weights),
//                      begin(weights_sorted), end(weights_sorted));
////    std::sort(weights.begin(), weights.end());
//    int e = weights_sorted.size() * 0.5;
//    while (weights_sorted.at(e) == weights_sorted.at(e-1)) e--;
//    for (size_t i = 0; i < sample.range_data.returns.size(); ++i) {
//      if (weights.at(i) >= weights_sorted.at(e)) {
//        gnc_point_cloud.push_back(sample.range_data.returns[i]);
//      }
//    }
//    LOG(INFO) << "Removed points: "
//              << sample.range_data.returns.size() - gnc_point_cloud.size();
////    matched_pose_estimate = cartographer::transform::Rigid2<double>();  // reset?
////    cartographer::mapping::scan_matching::CeresScanMatcher2D scan_matcher_tsdf(ceres_scan_matcher_options);
////    scan_matcher_tsdf.Match(target_translation, initial_pose_estimate,
////                            gnc_point_cloud, grid, &matched_pose_estimate,
////                            &summary);
////    c->get_gnc_callback()->set_weights(weights);
//    sample.range_data.returns = gnc_point_cloud;
//  }

  auto match_end = std::chrono::high_resolution_clock::now();
  LOG(INFO) << "Estimated time: "
    << std::chrono::duration_cast<std::chrono::microseconds>(
        match_end - match_start).count() / 1e3 << " ms, " << summary.iterations.size();
//  LOG(INFO) << "Estimated Pose: " << matched_pose_estimate;
//  LOG(INFO) << summary.FullReport();
  const auto initial_error = initial_pose_estimate * sample.ground_truth_pose;
  const auto matching_error = matched_pose_estimate * sample.ground_truth_pose;
  sample_result->initial_trans_error = initial_error.translation().norm();
  sample_result->matching_trans_error = matching_error.translation().norm();
  sample_result->initial_rot_error = initial_error.rotation().smallestAngle();
  sample_result->matching_rot_error = matching_error.rotation().smallestAngle();
  sample_result->matching_iterations =
      summary.num_successful_steps + summary.num_unsuccessful_steps;
  sample_result->matching_time = summary.minimizer_time_in_seconds;
  //  LOG(INFO) << "Matching error " << sample_result->matching_trans_error << "
  //  \t"
  //            << sample_result->initial_trans_error << " \t"
  //            << sample_result->matching_iterations << " \t"
  //            << sample_result->matching_time;


  const std::vector<double> *gnc_weights;
  if (instanceof<cartographer::mapping::scan_matching::CeresScanMatcherGnc2D>
      (&scan_matcher)) {
    auto *c = dynamic_cast<cartographer::mapping::scan_matching::CeresScanMatcherGnc2D*>(&scan_matcher);
    gnc_weights = c->get_gnc_callback()->get_weights();
//    LOG(INFO) << "First 100 Weights: " << std::accumulate(gnc_weights->begin(), gnc_weights->begin()+ 100, 0.0);
//    LOG(INFO) << "Noise     Weights: " << std::accumulate(gnc_weights->begin() + 100, gnc_weights->end(), 0.0);
  } else {
    gnc_weights = nullptr;
  }

  // TODO remove comment
  /*if (sample_result->matching_iterations > 0 &&
      sample_result->matching_trans_error < 0.01) {
//    LOG(INFO) << summary.FullReport();
    LOG(INFO) << summary.inner_iteration_time_in_seconds;
    renderGridwithScan(grid, sample, initial_pose_estimate,
                       matched_pose_estimate, options, gnc_weights);
    std::vector<std::vector<double>> gradients;
    //      EvaluateCostFunctionSinglePoint(grid,options,
    //    ceres_scan_matcher_options, &gradients);
    EvaluateCostFunction<GridType, ScanMatcher>(
        sample, grid, options, ceres_scan_matcher_options, &gradients);
    std::ofstream log_file;
    time_t seconds;
    time(&seconds);
    std::string grid_type;
    switch (grid.GetGridType()) {
      case cartographer::mapping::GridType::PROBABILITY_GRID:
        grid_type = "pg";
        break;
      case cartographer::mapping::GridType::TSDF:
        grid_type = "tsdf";
        break;
    }
    std::string log_file_path =
        "gradient_" + grid_type + "_" +
        std::to_string(sample_result->matching_iterations) + ".csv";
    log_file.open(log_file_path);
    for (auto& row : gradients) {
      for (auto& element : row) {
        log_file << element << ",";
      }
      log_file << "\n";
    }
    log_file.close();
  }*/
  if (do_render)
    renderGridwithScan(grid, sample, initial_pose_estimate,
                       matched_pose_estimate, options, gnc_weights);
}

template <typename GridType, typename RangeDataInserter,
          typename CeresScanMatcher>
void EvaluateScanMatcher(
    const std::vector<Sample>& training_set,
    const std::vector<Sample>& test_set,
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    const cartographer::mapping::scan_matching::proto::
        CeresScanMatcherOptions2D& ceres_scan_matcher_options,
    std::vector<SampleResult>* results) {
  std::unique_ptr<RangeDataInserter> range_data_inserter =
      generateRangeDataInserter<RangeDataInserter>(range_data_inserter_options);
  std::unique_ptr<GridType> grid =
      generateGrid<GridType>(range_data_inserter_options);

  for (auto sample : training_set) {
    range_data_inserter->Insert(sample.range_data, grid.get());
  }

  for (auto sample : test_set) {
    SampleResult sample_result_gnc;
    MatchScan<GridType, CeresScanMatcher>(
        sample, ceres_scan_matcher_options,
        range_data_inserter_options,
        *grid.get(), &sample_result_gnc);
    results->push_back(sample_result_gnc);
  }

}

template <typename GridType, typename RangeDataInserter>
void EvaluateScanMatcherGradient(
    const std::vector<Sample>& training_set,
    const std::vector<Sample>& test_set,
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    const cartographer::mapping::scan_matching::proto::
        CeresScanMatcherOptions2D& ceres_scan_matcher_options,
    std::vector<std::vector<double>>* gradients) {
  std::unique_ptr<RangeDataInserter> range_data_inserter =
      generateRangeDataInserter<RangeDataInserter>(range_data_inserter_options);

  std::unique_ptr<GridType> grid =
      generateGrid<GridType>(range_data_inserter_options);

  for (auto sample : training_set) {
    range_data_inserter->Insert(sample.range_data, grid.get());
  }
  float min_trans = -0.3;
  float max_trans = 0.3;
  float resolution_trans = 0.025;
  float min_rot = 0.f;
  float max_rot =  1E-7f;  // 1E-7f;  // 2.f * M_PI;
  float resolution_rot =
      resolution_trans * (max_rot - min_rot) / (max_trans - min_trans);
  for (auto sample : test_set) {
    for (float x = min_trans; x < max_trans; x += resolution_trans) {
      for (float y = min_trans; y < max_trans; y += resolution_trans) {
        for (float theta = min_rot; theta < max_rot; theta += resolution_rot) {
          cartographer::mapping::scan_matching::CeresScanMatcherGnc2D scan_matcher(
              ceres_scan_matcher_options);
          const Eigen::Vector2d target_translation = {0., 0.};
          const cartographer::transform::Rigid2d initial_pose_estimate =
              cartographer::transform::Rigid2d({x, y}, theta);
          cartographer::transform::Rigid2d matched_pose_estimate;
          ceres::Solver::Summary summary;
          double cost;
          std::vector<double> jacobians;

          scan_matcher.Evaluate(target_translation, initial_pose_estimate,
                                sample.range_data.returns, *grid.get(), &cost,
                                NULL, &jacobians);

          std::vector<double> result = {x, y, theta, cost};
          result.insert(result.end(), jacobians.begin(), jacobians.end());
          gradients->push_back(result);
        }
      }
      LOG(INFO) << 100.0 * (1.0 - (max_trans - x) / (max_trans - min_trans))
                << "%";
    }
  }
}

template <typename GridType>
void EvaluateCostFunctionSinglePoint(
    const GridType& grid,
    const cartographer::mapping::proto::RangeDataInserterOptions&
        range_data_inserter_options,
    const cartographer::mapping::scan_matching::proto::
        CeresScanMatcherOptions2D& ceres_scan_matcher_options,
    std::vector<std::vector<double>>* gradients) {
  Sample sample = Sample();

  sample.range_data.returns.push_back({Eigen::Vector3f(0.f, 0.f, 0.f)});
  sample.range_data.origin = Eigen::Vector3f{0, 0, 0};

  float min_trans = -1.0f;
  float max_trans = 1.0f;
  float resolution_trans = 0.01;
  float min_rot = -01.f;
  float max_rot = 0.1;  // 1E-7f;  // 2.f * M_PI;
  float resolution_rot =
      resolution_trans * (max_rot - min_rot) / (max_trans - min_trans);
  for (float x = min_trans; x <= max_trans; x += resolution_trans) {
    for (float y = min_trans; y <= max_trans; y += resolution_trans) {
      for (float theta = min_rot; theta <= max_rot; theta += resolution_rot) {
        // float theta = min_rot;
        cartographer::mapping::scan_matching::CeresScanMatcherGnc2D scan_matcher(
            ceres_scan_matcher_options);
        const Eigen::Vector2d target_translation = {0., 0.};
        const cartographer::transform::Rigid2d initial_pose_estimate =
            cartographer::transform::Rigid2d({x, y}, theta);
        cartographer::transform::Rigid2d matched_pose_estimate;
        ceres::Solver::Summary summary;
        double cost;
        std::vector<double> jacobians;

        scan_matcher.Evaluate(target_translation, initial_pose_estimate,
                              sample.range_data.returns, grid, &cost, NULL,
                              &jacobians);

        std::vector<double> result = {x, y, theta, cost};
        result.insert(result.end(), jacobians.begin(), jacobians.end());
        gradients->push_back(result);
      }
    }
    //    LOG(INFO) << 100.0 * (1.0 - (max_trans - x) / (max_trans - min_trans))
    //              << "%";
  }
}

void RunScanMatchingEvaluation() {
  cartographer::mapping::proto::RangeDataInserterOptions
      range_data_inserter_options;
  auto parameter_dictionary_range_data_inserter = common::MakeDictionary(
      "return { "
      "range_data_inserter_type = \"PROBABILITY_GRID_INSERTER_2D\","
      "probability_grid_range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.7, "
      "miss_probability = 0.4, "
      "},"
      "tsdf_range_data_inserter = {"
      "truncation_distance = 0.25,"
      "truncation_distance_update_factor = 1.0,"
      "update_free_space_only_first_hits = false,"
      "maximum_weight = 128.,"
      "update_free_space = true,"
      "normal_estimation_options = {"
      "num_normal_samples = 16,"
      "sample_radius = 0.3,"
      "tsdf_weight_scale = 1.0,"
      "const_weight = 0.1,"
      "sort_range_data = true,"
      "use_pca = false,"
      "},"
      "free_space_weight = 0.3,"
      "project_sdf_distance_to_scan_normal = false,"
      "update_weight_range_exponent = 0,"
      "update_weight_angle_scan_normal_to_ray_kernel_bandwith = 0.0,"
      "update_weight_distance_cell_to_hit_kernel_bandwith = 10.0,"
      "min_normal_weight = 0.1,"
      "},"
      "}");
  range_data_inserter_options =
      cartographer::mapping::CreateRangeDataInserterOptions(
          parameter_dictionary_range_data_inserter.get());
//          occupied_space_weight = 1.,
  auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 0.1,
          translation_weight = 0.001,
          translation_weight_vertical = 0.0,
          rotation_weight = 0.001,
          empty_space_cost = 0.5,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 500,
            num_threads = 1,
          },
          gnc_options_2d = {
            use_gnc = true,
            max_iterations = 80,
            non_convexity_stop = 0.01,
            gm_shape = 32,
            min_convexity = 10,
            non_convexity_inc_factor = 1.4,
            max_retries = 1,
          },
        })text");
  cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      ceres_scan_matcher_options =
          cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
              parameter_dictionary.get());

  std::ofstream log_file;
  std::string log_file_path;
  time_t seconds;
  time(&seconds);
  log_file_path = "scan_matching_benchmark" + std::to_string(seconds) + ".csv";
  log_file.open(log_file_path);
  log_file << "grid_type,grid_resolution,outlier,sparseness,resolution,"
              "cloud_noise,initial_error_trans,initial_error_angle,"
              "matched_error_trans,matched_error_angle,solver_iterations,"
              "matching_time,gm_shape,min_convexity,non_convexity_inc_factor,"
              "max_retry,size_x,size_y\n";

  //  std::vector<double> trans_errors = {0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3,
  //  0.35, 0.4, 0.45, 0.5};
  //  std::vector<double> rot_errors = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
  //  0.8};

//    std::vector<double> trans_errors = {0.0,  0.1, 0.2, 0.3, 0.4};
//    std::vector<double> rot_errors = {0.0, 0.2, 0.4, 0.6, 0.8};
  //  std::vector<double> trans_errors = {0.0, 0.05};
//  std::vector<double> trans_errors = {0.0};

  const std::vector<double> trans_errors = {0.2};
  const std::vector<double> rot_errors = {0.2};
  const std::vector<double> outlier_ratios = {0.3};
//  const std::vector<double> outlier_ratios = {0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,0.95};
  const std::vector<double> sparse_ratios = {0.0};
  const std::vector<double> resolutions = {0.025};  // 0.025
  const std::vector<double> cloud_noises = {0.01};  // 0.01
  const std::vector<double> gm_shapes = {7};  // LIDAR: 3.57143  // TODO NON CONVEXITY STOP IS SET TO 1?
  const std::vector<double> min_convexities = {1};
  const std::vector<double> non_convexity_inc_factors = {1.2};  // LIDAR: 1.04664
  const std::vector<double> max_retries = {1};
  do_render = false;
//  float outlier_ratio = 3.2;
//  float sparse_ratio = 0.0;
//  const double resolution = 0.045;
//  const float cloud_noise = 0.0225;
  int n_training = 1;
  int n_test = 1;
  float size_x = 1.025;
  float size_y = 1.025;
//  float base_size = 1.0;
  int num_shapes = 1;
  int num_repetitions = 100;
  for (int square_idx = 0; square_idx < num_shapes; ++square_idx) {
//    float offset =
//        float(square_idx) * float(0.05f) / float(num_shapes) - 0.5f * 0.05f;
    float offset = 0.5;
    if (num_shapes == 1) offset = 0.f;
    size_x = size_x + offset;
    size_y = size_y + offset;

    for (int i_rep = 0; i_rep < num_repetitions; ++i_rep) {
      for (double error_trans : trans_errors) {
      for (double error_rot : rot_errors) {
      for (double outlier_ratio : outlier_ratios) {
      for (double sparse_ratio : sparse_ratios) {
      for (double resolution : resolutions) {
      for (double cloud_noise : cloud_noises) {
      for (double gm_shape : gm_shapes) {
      for (double min_convexity : min_convexities) {
      for (double non_convexity_inc_factor : non_convexity_inc_factors) {
      for (double max_retry : max_retries) {
        LOG(INFO) << "Current Repetition: " << i_rep << " of " << num_repetitions;
        ceres_scan_matcher_options.mutable_gnc_options_2d()
          ->set_gm_shape(gm_shape);
        ceres_scan_matcher_options.mutable_gnc_options_2d()
            ->set_min_convexity(min_convexity);
        ceres_scan_matcher_options.mutable_gnc_options_2d()
            ->set_non_convexity_inc_factor(non_convexity_inc_factor);
        ceres_scan_matcher_options.mutable_gnc_options_2d()
            ->set_max_retries(max_retry);
//        const ScanCloudGenerator::ModelType model_type = ScanCloudGenerator::ModelType::RECTANGLE_RADAR;
        const ScanCloudGenerator::ModelType model_type = ScanCloudGenerator::ModelType::RECTANGLE;
        Eigen::Vector2d size = {size_x, size_y};
        auto SG = SampleGenerator(n_training, n_test, error_trans, error_rot,
                                  outlier_ratio, sparse_ratio);
//        std::string csv_path = "/home/mo/drz/src/radar_filter/recorded_data/pc2_data/2020_11_04/16_44_24_300points_radar_pcl_gnc.csv";

//          std::vector<Sample> training_set;
//          training_set.push_back(SG.GenerateSample(0.0, 0.0, csv_path, 0, 30));

//          std::vector<Sample> test_set;
//          test_set.push_back(SG.GenerateSample(error_trans, error_rot, csv_path, 31, 1));
//          test_set.push_back(SG.GenerateSample(Eigen::Vector3f(1.39,0,0)));

        SG.GenerateSampleSet(model_type, size, resolution, cloud_noise, outlier_ratio);
        std::vector<Sample> training_set = SG.getTrainingSet();
//          SG.GenerateSampleSet(csv_path, 1, 2);
//        cartographer::sensor::RangefinderPoint a;
//        a.position = Eigen::Vector3f(0,3,0);
//        training_set.at(0).range_data.returns.push_back(a);
//        Eigen::Vector2d size_2 = {size_x + 0.6, size_y + 0.6};
//        auto SG2 = SampleGenerator(n_training, n_test, error_trans, error_rot,
//                                   outlier_ratio, sparse_ratio);
//        SG2.GenerateSampleSet(model_type, size_2, resolution, cloud_noise, outlier_ratio);
//        std::vector<Sample> training_set_2 = SG2.getTrainingSet();
//        training_set.at(0).range_data.returns.insert(training_set.at(0).range_data.returns.end(), training_set_2.at(0).range_data.returns.begin(), training_set_2.at(0).range_data.returns.end());

//        std::vector<Sample> training_set = SG.getTrainingSet();
//        SG.GenerateSampleSet(model_type, size, resolution, cloud_noise);
        std::vector<Sample> test_set = SG.getTestSet(); // TODO dont copy
//        std::vector<Sample> test_set = SG.getTrainingSet(); // TODO dont copy
//        std::vector<Sample> test_set = training1_set; // TODO dont copy


//        std::vector<Sample> test_set;
//        std::vector<Eigen::Vector3f> points;
//        std::normal_distribution<float> normal_distribution(0, cloud_noise);
//        std::default_random_engine e1(42);
//        auto center = Eigen::Vector3f(0.0, 0.0, 0.0);
//        points.push_back(Eigen::Vector3f(-0.35, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.30, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.25, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.20, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.15, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.10, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.05, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.00, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.05, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.10, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.15, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.20, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.25, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.30, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.35, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.40, 1.0 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.0, 0.9 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.0, 1.1 + normal_distribution(e1), 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.015, 2, 0.0));
//        points.push_back(Eigen::Vector3f(-0.015, 2, 0.0));


//        auto center = Eigen::Vector3f(0.0, 1.0, 0.0);
//        srand(1);
//        for (int i=-8; i<8; i++) {
//          points.push_back(Eigen::Vector3f(-float(i) / 10.0, 1.0 + (double)rand() / (double)RAND_MAX / 100.0, 0.0) + center);
//        }
//        points.push_back(Eigen::Vector3f(-0.01, 1.15, 0.0) + center);
//        points.push_back(Eigen::Vector3f(-0.05, 0.89, 0.0) + center);
//        points.push_back(Eigen::Vector3f(0.05, 0.89, 0.0) + center);
//        for (int i=0; i<7; i++) {
//          auto center = Eigen::Vector3f(0.6, 0.2, 0.0);
//          points.push_back(Eigen::Vector3f((double)rand() / (double)RAND_MAX / 10, (double)rand() / (double)RAND_MAX / 10, 0.0) + center);
//        }
//          std::vector<Eigen::Vector3f> points = {
//              Eigen::Vector3f(0.0, 0.0, 0.0)
//              Eigen::Vector3f(0.0, 0.5, 0.0),
//              Eigen::Vector3f(0.0, 0.8, 0.0),
//              Eigen::Vector3f(0.0, 1.0, 0.0)
//          };

//        std::vector<Sample> training_set;
//        training_set.push_back(SG.GenerateSample(points));
//        std::vector<Eigen::Vector3f> points_test;
////        points_test.push_back(Eigen::Vector3f(0.0, 0.0, 0.0));
////        test_set.push_back(SG.GenerateSample(points_test));
//        test_set.push_back(SG.GenerateSample(points));

          LOG(INFO) << "STD DEV: " << test_set[0].std_dev_x << ", "
                    << test_set[0].std_dev_y << ", " << test_set[0].std_dev_xy
                    << ", e: " << test_set[0].error_sqr;

        //      Sample sample;
        //      sample.ground_truth_pose =
        //      cartographer::transform::Rigid2d({0,
        //      0}, 0);
        //      sensor::RangeData initial_pose_estimate_range_data;
        //      cartographer::sensor::PointCloud returns;
        //      returns.push_back({Eigen::Vector3f(-0.1, 0.5, 0)});
        //      returns.push_back({Eigen::Vector3f(-0.05, 0.5, 0)});
        //      returns.push_back({Eigen::Vector3f(0.0, 0.5, 0)});
        //      returns.push_back({Eigen::Vector3f(0.05, 0.5, 0)});
        //      returns.push_back({Eigen::Vector3f(0.1, 0.5, 0)});
        //      initial_pose_estimate_range_data.origin = Eigen::Vector3f{0,
        //      0,
        //      0};
        //      initial_pose_estimate_range_data.returns = returns;
        //      sample.range_data = initial_pose_estimate_range_data;
        //      training_set.push_back(sample);
        //      test_set.push_back(sample);

        LOG(INFO) << "Translation: " << error_trans << ", Rotation: "
                  << error_rot << ", Outlier: " << outlier_ratio
                  << ", Sparse: " << sparse_ratio;
        double avg_matching_trans_error = 0.0;
        double avg_matching_rot_error = 0.0;

//        // Probability Grid
//        std::vector<SampleResult> probability_grid_results;
//        LOG(INFO) << "Evaluating probability grid:";
//        EvaluateScanMatcher<
//            cartographer::mapping::ProbabilityGrid,
//            cartographer::mapping::ProbabilityGridRangeDataInserter2D,
//            cartographer::mapping::scan_matching::CeresScanMatcher2D>(
//            training_set, test_set, range_data_inserter_options,
//            ceres_scan_matcher_options, &probability_grid_results);
//        for (const auto& res : probability_grid_results) {
//          log_file << "PROBABILITY_GRID"
//                   << "," << resolution << "," << res.initial_trans_error
//                   << "," << res.initial_rot_error << ","
//                   << res.matching_trans_error << ","
//                   << res.matching_rot_error << "," << res.matching_iterations
//                   << "," << res.matching_time << "," << size[0] << ","
//                   << size[1] << "\n";
//          avg_matching_trans_error +=
//              res.matching_trans_error / probability_grid_results.size();
//          avg_matching_rot_error += std::abs(res.matching_rot_error /
//                                             probability_grid_results.size());
//        }
//        LOG(INFO) << "PG avg_matching_trans_error \t"
//                  << avg_matching_trans_error;
//        LOG(INFO) << "PG avg_matching_rot_error \t" << avg_matching_rot_error;
//
//          // Probability Grid GNC
//          std::vector<SampleResult> probability_grid_results_gnc;
//          LOG(INFO) << "Evaluating probability grid:";
//          EvaluateScanMatcher<
//              cartographer::mapping::ProbabilityGrid,
//              cartographer::mapping::ProbabilityGridRangeDataInserter2D,
//              cartographer::mapping::scan_matching::CeresScanMatcherGnc2D>(
//              training_set, test_set, range_data_inserter_options,
//              ceres_scan_matcher_options, &probability_grid_results_gnc);
//          for (const auto& res : probability_grid_results_gnc) {
//            log_file << "PROBABILITY_GRID"
//                     << "," << resolution << "," << res.initial_trans_error
//                     << "," << res.initial_rot_error << ","
//                     << res.matching_trans_error << ","
//                     << res.matching_rot_error << "," << res.matching_iterations
//                     << "," << res.matching_time << "," << size[0] << ","
//                     << size[1] << "\n";
//            avg_matching_trans_error +=
//                res.matching_trans_error / probability_grid_results_gnc.size();
//            avg_matching_rot_error += std::abs(res.matching_rot_error /
//                                                   probability_grid_results_gnc.size());
//          }
//          LOG(INFO) << "PG GNC avg_matching_trans_error \t"
//                    << avg_matching_trans_error;
//          LOG(INFO) << "PG GNC avg_matching_rot_error \t" << avg_matching_rot_error;

        // TSDF Grid Normal
        std::vector<SampleResult> tsdf_normal_results;
//            LOG(INFO) << "Evaluating normal TSDF grid:";
        EvaluateScanMatcher<cartographer::mapping::TSDF2D, cartographer::mapping::TSDFRangeDataInserter2D, cartographer::mapping::scan_matching::CeresScanMatcher2D>(
            training_set, test_set, range_data_inserter_options,
            ceres_scan_matcher_options, &tsdf_normal_results);
        avg_matching_trans_error = 0.0;
        avg_matching_rot_error = 0.0;
        for (const auto &res : tsdf_normal_results) {
          log_file << "Normal TSDF" << "," << resolution
                   << "," << outlier_ratio << "," << sparse_ratio << ","
                   << resolution << "," << cloud_noise << ","
                   << res.initial_trans_error << "," << res.initial_rot_error << ","
                   << res.matching_trans_error << "," << res.matching_rot_error
                   << "," << res.matching_iterations << "," << res.matching_time
                   << "," << gm_shape << "," << min_convexity << ","
                   << non_convexity_inc_factor << "," << max_retry << ","
                   << size[0] << "," << size[1] << "\n";
          avg_matching_trans_error +=
              res.matching_trans_error / tsdf_normal_results.size();
          avg_matching_rot_error += std::abs(
              res.matching_rot_error / tsdf_normal_results.size());
        }
        LOG(INFO) << "Normal TSDF avg_matching_trans_error \t"
                  << avg_matching_trans_error;
        LOG(INFO) << "Normal TSDF avg_matching_rot_error \t"
                  << avg_matching_rot_error;
        LOG(INFO) << "\n";

        // TSDF Grid GNC
        std::vector<SampleResult> tsdf_results;
//            LOG(INFO) << "Evaluating GNC TSDF:";
        EvaluateScanMatcher<cartographer::mapping::TSDF2D, cartographer::mapping::TSDFRangeDataInserter2D, cartographer::mapping::scan_matching::CeresScanMatcherGnc2D>(
            training_set, test_set, range_data_inserter_options,
            ceres_scan_matcher_options, &tsdf_results);
        avg_matching_trans_error = 0.0;
        avg_matching_rot_error = 0.0;
        for (const auto &res : tsdf_results) {
          log_file << "GNC TSDF" << "," << resolution
              << "," << outlier_ratio << "," << sparse_ratio << ","
              << resolution << "," << cloud_noise << ","
              << res.initial_trans_error << "," << res.initial_rot_error << ","
              << res.matching_trans_error << "," << res.matching_rot_error
              << "," << res.matching_iterations << "," << res.matching_time
              << "," << gm_shape << "," << min_convexity << ","
              << non_convexity_inc_factor << "," << max_retry << ","
              << size[0] << "," << size[1] << "\n";
          avg_matching_trans_error +=
              res.matching_trans_error / tsdf_results.size();
          avg_matching_rot_error += std::abs(
              res.matching_rot_error / tsdf_results.size());
        }
        LOG(INFO) << "GNC TSDF avg_matching_trans_error \t"
                  << avg_matching_trans_error;
        LOG(INFO) << "GNC TSDF avg_matching_rot_error \t"
                  << avg_matching_rot_error;
        LOG(INFO) << "\n";

//        std::vector<std::vector<double>> gradients;
//        EvaluateScanMatcherGradient<
//            cartographer::mapping::TSDF2D,
//            cartographer::mapping::TSDFRangeDataInserter2D>(
//                training_set, test_set, range_data_inserter_options,
//                ceres_scan_matcher_options, &gradients
//                );
      }
      }
      }
      }
      }
      }
      }
      }
      }
      }
    }
  }
  log_file.close();
}

void RunScanMatchingConvergenceWindowEvaluation() {
  cartographer::mapping::proto::RangeDataInserterOptions
      range_data_inserter_options;
  auto parameter_dictionary_range_data_inserter = common::MakeDictionary(
      "return { "
      "range_data_inserter_type = \"PROBABILITY_GRID_INSERTER_2D\","
      "probability_grid_range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.7, "
      "miss_probability = 0.4, "
      "},"
      "tsdf_range_data_inserter = {"
      "truncation_distance = 0.4,"
      "truncation_distance_update_factor = 1.0,"
      "maximum_weight = 300.,"
      "update_free_space = true,"
      "normal_estimation_options = {"
      "num_normal_samples = 16,"
      "sample_radius = 0.15,"
      "tsdf_weight_scale = 0.0,"
      "const_weight = 0.1,"
      "sort_range_data = true,"
      "use_pca = false,"
      "},"
      "free_space_weight = 0.3,"
      "project_sdf_distance_to_scan_normal = true,"
      "update_weight_range_exponent = 0,"
      "update_weight_angle_scan_normal_to_ray_kernel_bandwith = 0.0,"
      "update_weight_distance_cell_to_hit_kernel_bandwith = 0.0,"
      "min_normal_weight = 0.1,"
      "},"
      "}");
  range_data_inserter_options =
      cartographer::mapping::CreateRangeDataInserterOptions(
          parameter_dictionary_range_data_inserter.get());
  auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 0.25,
          translation_weight = 0.0,
          rotation_weight = 0.0,
          empty_space_cost = 1.0,
          ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 500,
            num_threads = 1,
          },
          gnc_options_2d = {
            max_iterations = 80,
            max_iterations = 1.0,
            max_iterations = 9.35,
            max_iterations = 10,
            max_iterations = 1.4,
          },
        })text");
  const cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      ceres_scan_matcher_options =
          cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
              parameter_dictionary.get());
  int n_training = 100;
  int n_test = 25;

  std::ofstream log_file;
  std::string log_file_path;
  time_t seconds;
  time(&seconds);
  log_file_path = "scan_matching_benchmark" + std::to_string(seconds) + ".csv";
  log_file.open(log_file_path);
  log_file << "grid_type,grid_resolution,initial_error_trans,initial_error_"
              "angle,matched_error_trans,matched_error_angle,solver_iterations,"
              "matching_time,size_x,size_y\n";

  //  std::vector<double> trans_errors = {0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3,
  //  0.35, 0.4, 0.45, 0.5};
  //  std::vector<double> rot_errors = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
  //  0.8};

  //  std::vector<double> trans_errors = {0.0,  0.1, 0.2, 0.3, 0.4};
  //  std::vector<double> rot_errors = {0.0, 0.2, 0.4, 0.6, 0.8};
//  std::vector<double> trans_errors = {0.1, 0.2, 0.35, 0.4, 0.45, 0.5};
  //  std::vector<double> trans_errors = {0.0};

  std::vector<double> trans_errors = {0.1};
  std::vector<double> rot_errors = {0.5};
//  const double resolution = 0.025;
//  const float cloud_noise = 0.01;
  float size_x = 1.0;
  float size_y = 1.0;
  float base_size = 1.0;
  int num_shapes = 1;
  int num_repetitions = 1;
  for (int square_idx = 0; square_idx < num_shapes; ++square_idx) {
    float offset =
        float(square_idx) * float(0.05f) / float(num_shapes) - 0.5f * 0.05f;
    if (num_shapes == 1) offset = 0.f;
    size_x = base_size + offset;
    size_y = base_size + offset;

    for (int i_rep = 0; i_rep < num_repetitions; ++i_rep) {
//      const ScanCloudGenerator::ModelType model_type =
//          ScanCloudGenerator::ModelType::RECTANGLE;
      Eigen::Vector2d size = {size_x, size_y};
      const double resolution = 0.025;
//      const float cloud_noise = 0.01;  // 0.025;
      auto SG = SampleGenerator(n_training, n_test, 0., 0., 0.0);
      std::string csv_path = "/home/mo/drz/src/radar_filter/recorded_data/pc2_data/2020_11_04/16_44_24_300points_radar_pcl_gnc.csv";
      SG.GenerateSampleSet(csv_path, 1, 2);
//      SG.GenerateDefinedSampleSet(model_type, size, resolution, cloud_noise);
      std::vector<Sample> training_set = SG.getTrainingSet();
      std::vector<Sample> test_set = SG.getTestSet();

      //      Sample sample;
      //      sample.ground_truth_pose =
      //      cartographer::transform::Rigid2d({0,
      //      0}, 0);
      //      sensor::RangeData initial_pose_estimate_range_data;
      //      cartographer::sensor::PointCloud returns;
      //      returns.push_back({Eigen::Vector3f(-0.1, 0.5, 0)});
      //      returns.push_back({Eigen::Vector3f(-0.05, 0.5, 0)});
      //      returns.push_back({Eigen::Vector3f(0.0, 0.5, 0)});
      //      returns.push_back({Eigen::Vector3f(0.05, 0.5, 0)});
      //      returns.push_back({Eigen::Vector3f(0.1, 0.5, 0)});
      //      initial_pose_estimate_range_data.origin = Eigen::Vector3f{0,
      //      0,
      //      0};
      //      initial_pose_estimate_range_data.returns = returns;
      //      sample.range_data = initial_pose_estimate_range_data;
      //      training_set.push_back(sample);
      //      test_set.push_back(sample);

      std::vector<SampleResult> probability_grid_results;
      LOG(INFO) << "Evaluating probability grid:";
      EvaluateScanMatcher<
        cartographer::mapping::ProbabilityGrid,
        cartographer::mapping::ProbabilityGridRangeDataInserter2D,
        cartographer::mapping::scan_matching::CeresScanMatcher2D>(
          training_set, test_set, range_data_inserter_options,
          ceres_scan_matcher_options, &probability_grid_results);
      double avg_matching_trans_error = 0.0;
      double avg_matching_rot_error = 0.0;
      for (const auto& res : probability_grid_results) {
        log_file << "PROBABILITY_GRID"
                 << "," << resolution << "," << res.initial_trans_error << ","
                 << res.initial_rot_error << "," << res.matching_trans_error
                 << "," << res.matching_rot_error << ","
                 << res.matching_iterations << "," << res.matching_time << ","
                 << size[0] << "," << size[1] << "\n";
        avg_matching_trans_error +=
            res.matching_trans_error / probability_grid_results.size();
        avg_matching_rot_error +=
            std::abs(res.matching_rot_error / probability_grid_results.size());
      }
      LOG(INFO) << "PG avg_matching_trans_error \t" << avg_matching_trans_error;
      LOG(INFO) << "PG avg_matching_rot_error \t" << avg_matching_rot_error;
      std::vector<SampleResult> tsdf_results;
      LOG(INFO) << "Evaluating TSDF:";
      EvaluateScanMatcher<cartographer::mapping::TSDF2D,
        cartographer::mapping::TSDFRangeDataInserter2D,
        cartographer::mapping::scan_matching::CeresScanMatcher2D>,(
          training_set, test_set, range_data_inserter_options,
          ceres_scan_matcher_options, &tsdf_results);
      avg_matching_trans_error = 0.0;
      avg_matching_rot_error = 0.0;
      for (const auto& res : tsdf_results) {
        log_file << "TSDF"
                 << "," << resolution << "," << res.initial_trans_error << ","
                 << res.initial_rot_error << "," << res.matching_trans_error
                 << "," << res.matching_rot_error << ","
                 << res.matching_iterations << "," << res.matching_time << ","
                 << size[0] << "," << size[1] << "\n";
        avg_matching_trans_error +=
            res.matching_trans_error / tsdf_results.size();
        avg_matching_rot_error +=
            std::abs(res.matching_rot_error / tsdf_results.size());
      }
      LOG(INFO) << "TSDF avg_matching_trans_error \t"
                << avg_matching_trans_error;
      LOG(INFO) << "TSDF avg_matching_rot_error \t" << avg_matching_rot_error;
      LOG(INFO) << "\n";
    }
  }
  log_file.close();
}

}  // namespace evaluation
}  // namespace cartographer


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
//  auto points = load_csv("/home/mo/drz/src/radar_filter/recorded_data/pc2_data/2020_11_04/16_44_24_300points_radar_pcl_gnc.csv", 1, 1);
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);
  cartographer::evaluation::RunScanMatchingEvaluation();
}
