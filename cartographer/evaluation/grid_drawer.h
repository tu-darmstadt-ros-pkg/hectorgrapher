#ifndef CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
#define CARTOGRAPHER_EVALUATION_GRID_DRAWER_H

#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer/sensor/point_cloud.h>
#include "cartographer/mapping/2d/edf_2d.h"
#include "cartographer/mapping/2d/tsdf_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_esdf_scan_matcher_2d.h"

#include <cairo/cairo-svg.h>
//#include "cairo-svg.h"
//#include "cairo.h"

namespace cartographer {
namespace evaluation {

class GridDrawer {
 public:
  GridDrawer(const cartographer::mapping::MapLimits& limits);
  void DrawTSD(const cartographer::mapping::TSDF2D& grid);
  void DrawED(const cartographer::mapping::EDF2D& grid);
  void DrawWeights(const cartographer::mapping::TSDF2D& grid);

  void DrawScan(const sensor::RangeData& range_data,
                const cartographer::transform::Rigid2d& initial_transform,
                const cartographer::transform::Rigid2d& matched_transform);

  void DrawPointcloud(
      const sensor::PointCloud& range_data,
      const cartographer::transform::Rigid2d& initial_transform,
      const cartographer::transform::Rigid2d& matched_transform);

  void DrawScanNormals(
      const sensor::RangeData& range_data,
      const cartographer::transform::Rigid2d& transform,
      const cartographer::mapping::proto::RangeDataInserterOptions& options);

  void DrawTSDFNormals(const cartographer::mapping::TSDF2D& grid,
                       const sensor::RangeData& range_data,
                       const cartographer::transform::Rigid2d& transform);

  void DrawWeightedTSDFNormals(
      const cartographer::mapping::TSDF2D& grid,
      const sensor::RangeData& range_data,
      const cartographer::transform::Rigid2d& transform, float max_weight);

  void DrawWeightedScanNormals(
      const sensor::RangeData& range_data,
      const cartographer::transform::Rigid2d& transform,
  const cartographer::mapping::proto::TSDFRangeDataInserterOptions2D& options,
  float max_weight);

  void DrawWeightedNormals(std::vector<std::pair<float, float>> normals,
                       const sensor::RangeData& range_data,
                       const cartographer::transform::Rigid2d& transform, float max_weight);


  void DrawIsoSurface(const cartographer::mapping::TSDF2D& grid);
  void DrawBBBounds(
      const std::vector<
          cartographer::mapping::scan_matching::BBEvaluatedCandidates>
          candidates,
      const transform::Rigid2d& initial_pose_estimate);
  void ToFile(std::string filename);

 private:
  const cartographer::mapping::MapLimits& limits_;
  cairo_surface_t* grid_surface_;
  cairo_t* grid_surface_context_;
  double scale_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
