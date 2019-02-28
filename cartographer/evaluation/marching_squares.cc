#include "cartographer/evaluation/marching_squares.h"
#include <bitset>
#include "marching_squares.h"

namespace cartographer {
namespace evaluation {
namespace {
template <typename T>
size_t sgn(T val) {
  return (val > T(0));
}
}
std::vector<std::vector<Eigen::Vector2f>> computeSurfaces(
    const mapping::TSDF2D& grid) {
  std::vector<std::vector<Eigen::Vector2f>> surfaces;

  const cartographer::mapping::MapLimits& limits = grid.limits();
  double scale = 1. / limits.resolution();
  int scaled_num_x_cells = limits.cell_limits().num_x_cells * scale;
  int scaled_num_y_cells = limits.cell_limits().num_y_cells * scale;

  for (int ix = 0; ix < scaled_num_x_cells; ++ix) {
    for (int iy = 0; iy < scaled_num_y_cells; ++iy) {
      if ((grid.GetWeight({iy + 1, ix}) == 0.f) ||
          (grid.GetWeight({iy + 1, ix + 1}) == 0.f) ||
          (grid.GetWeight({iy, ix + 1}) == 0.f) ||
          (grid.GetWeight({iy, ix}) == 0.f)) {
        continue;
      }

      size_t pattern_code = 1 * sgn(grid.GetTSD({iy + 1, ix})) +
                            2 * sgn(grid.GetTSD({iy + 1, ix + 1})) +
                            4 * sgn(grid.GetTSD({iy, ix + 1})) +
                            8 * sgn(grid.GetTSD({iy, ix}));
      std::bitset<4> signs(pattern_code);

      if ((pattern_code == 0) || (pattern_code == 15)) {  // no contour
        continue;
      } else if ((pattern_code == 5) ||
                 (pattern_code == 10)) {  // two-segment saddle
        // TODO(kdaun)
        LOG(WARNING)
            << "computeSurfaces: two-segment saddle is not implemented";

      } else {  // single segment
        std::vector<Eigen::Vector2f> segment;
        std::bitset<4> directions(0);  // top, right, down, left
        if (signs[0] != signs[1]) {
          float t0 = grid.GetTSD({iy + 1, ix});
          float t1 = grid.GetTSD({iy + 1, ix + 1});
          Eigen::Vector2f p0 = grid.limits().GetCellCenter({iy + 1, ix});
          Eigen::Vector2f p1 = grid.limits().GetCellCenter({iy + 1, ix + 1});
          Eigen::Vector2f d = p1 - p0;
          segment.push_back(p0 - (t0 / (-t0 + t1)) * d);
        }
        if (signs[1] != signs[2]) {
          float t0 = grid.GetTSD({iy + 1, ix + 1});
          float t1 = grid.GetTSD({iy, ix + 1});
          Eigen::Vector2f p0 = grid.limits().GetCellCenter({iy + 1, ix + 1});
          Eigen::Vector2f p1 = grid.limits().GetCellCenter({iy, ix + 1});
          Eigen::Vector2f d = p1 - p0;
          segment.push_back(p0 - (t0 / (-t0 + t1)) * d);
        }
        if (signs[2] != signs[3]) {
          float t0 = grid.GetTSD({iy, ix + 1});
          float t1 = grid.GetTSD({iy, ix});
          Eigen::Vector2f p0 = grid.limits().GetCellCenter({iy, ix + 1});
          Eigen::Vector2f p1 = grid.limits().GetCellCenter({iy, ix});
          Eigen::Vector2f d = p1 - p0;
          segment.push_back(p0 - (t0 / (-t0 + t1)) * d);
        }
        if (signs[3] != signs[0]) {
          float t0 = grid.GetTSD({iy, ix});
          float t1 = grid.GetTSD({iy + 1, ix});
          Eigen::Vector2f p0 = grid.limits().GetCellCenter({iy, ix});
          Eigen::Vector2f p1 = grid.limits().GetCellCenter({iy + 1, ix});
          Eigen::Vector2f d = p1 - p0;
          segment.push_back(p0 - (t0 / (-t0 + t1)) * d);
        }
        surfaces.push_back(segment);
      }
    }
  }

  return surfaces;
}

}  // namespace evaluation
}  // namespace cartographer