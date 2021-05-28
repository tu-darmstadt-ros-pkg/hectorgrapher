#ifndef CARTOGRAPHER_TSDF_DRAWER_H
#define CARTOGRAPHER_TSDF_DRAWER_H

#include <fstream>
#include <string>

#include "cairo/cairo.h"

#ifdef WITH_OPEN3D
#include "open3d/Open3D.h"
#endif

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        class TSDFDrawer {
        public:
            TSDFDrawer();

            void drawTSDF(const std::shared_ptr<open3d::geometry::VoxelGrid> &voxelGridPointer);

            void saveSliceAsPNG(const int imageSliceIndex, const char* filename,const std::shared_ptr<open3d::geometry::VoxelGrid> &voxelGridPointerSlice);

        private:
            std::shared_ptr<open3d::geometry::VoxelGrid> tsdfPointer;
            int sliceIndex;
            int sliceOrientation;

            bool drawFullView(open3d::visualization::Visualizer *visualizer);

            bool changeOrientation(open3d::visualization::Visualizer *visualizer);

            bool loopThroughSlices(open3d::visualization::Visualizer *visualizer, int velocity);

            bool loopBackwardThroughSlices(open3d::visualization::Visualizer *visualizer);

            bool loopForwardThroughSlices(open3d::visualization::Visualizer *visualizer);
        };
    }
}

#endif //WITH_OPEN3D
#endif //CARTOGRAPHER_TSDF_DRAWER_H
