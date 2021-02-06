#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_GNC_COST_FUNCTION_2D_H
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_GNC_COST_FUNCTION_2D_H

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "cartographer/mapping/internal/2d/scan_matching/gnc_iteration_callback.h"


namespace cartographer {
namespace mapping {
namespace scan_matching {

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction* CreateOccupiedSpaceGncCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid, const GncIterationCallback* gnc_state);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif //CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_GNC_COST_FUNCTION_2D_H
