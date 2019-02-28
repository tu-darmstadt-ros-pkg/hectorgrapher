#ifndef CARTOGRAPHER_EVALUATION_MARCHING_SQUARES_H
#define CARTOGRAPHER_EVALUATION_MARCHING_SQUARES_H

#include <cartographer/mapping/2d/tsdf_2d.h>
namespace cartographer {
namespace evaluation {

// Computes a list of connected 0-surfaces of the TSDF using Marching Squares
// (https://en.wikipedia.org/wiki/Marching_squares).
std::vector<std::vector<Eigen::Vector2f>> computeSurfaces(
    const mapping::TSDF2D& grid);

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_MARCHING_SQUARES_H
