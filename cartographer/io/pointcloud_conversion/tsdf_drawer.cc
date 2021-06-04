#include "cartographer/io/pointcloud_conversion/tsdf_drawer.h"

#ifdef WITH_OPEN3D
namespace cartographer {
    namespace mapping {

        /**
         * Constructor for the TSDFDrawer-Class
         */
        TSDFDrawer::TSDFDrawer() {
            sliceIndex = 1;
            sliceOrientation = 2;       // The z-dimension is the default slice direction
        }

        /**
         * Show every voxel of a voxel grid with a z-index one higher than already seen.
         *
         * This method is a callback method for "drawTSDF".
         *
         * @param visualizer control class for the open3d window.
         * @return true.
         */
        bool TSDFDrawer::loopForwardThroughSlices(open3d::visualization::Visualizer *visualizer) {
            return loopThroughSlices(visualizer, -1);
        }

        /**
         * Show every voxel of a voxel grid with a z-index one lower than already seen.
         *
         * This method is a callback method for "drawTSDF".
         *
         * @param visualizer control class for the open3d window.
         * @return true.
         */
        bool TSDFDrawer::loopBackwardThroughSlices(open3d::visualization::Visualizer *visualizer) {
            return loopThroughSlices(visualizer, 1);
        }

        /**
         * Show every voxel of a voxel grid with a certain z-index. As a result, produce a "slice" of a grid.
         *
         * By copying the voxel size and origin point of the original voxel grid, the slice doesn't change position.
         * Since every call of this method includes an iteration of all voxels, this method could take a lot of
         * time when confronted with large areas or small voxels.
         *
         * @param visualizer control class for the open3d window.
         * @param velocity controls how the position of the slice is changed. Useful numbers are "1" or "-1".
         * @return true in any case.
         */
        bool TSDFDrawer::loopThroughSlices(open3d::visualization::Visualizer *visualizer, int velocity) {
            sliceIndex += velocity;
//                std::cout << "Show slice " << sliceIndex << std::endl;

            std::shared_ptr<open3d::geometry::VoxelGrid> slicedVoxelGridPointer =
                    std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());

            slicedVoxelGridPointer->voxel_size_ = tsdfPointer->voxel_size_;
            slicedVoxelGridPointer->origin_ = tsdfPointer->origin_;

            for (open3d::geometry::Voxel nextVoxel : tsdfPointer->GetVoxels()) {
                if (nextVoxel.grid_index_(sliceOrientation) == sliceIndex) {
                    slicedVoxelGridPointer->AddVoxel(nextVoxel);
                }
            }
            visualizer->ClearGeometries();
            visualizer->AddGeometry(slicedVoxelGridPointer, false);
            return true;
        }

        /**
         * Change the dimension of the displayed slices of a VoxelGrid.
         *
         * This method is a callback method for "drawTSDF".
         *
         * @param visualizer control class for the open3d window.
         * @return true in any case.
         */
        bool TSDFDrawer::changeOrientation(open3d::visualization::Visualizer *visualizer) {
            sliceOrientation = (sliceOrientation + 1) % 3;
            sliceIndex = 1;
            drawFullView(visualizer);
            return true;
        }

        /**
         * Show a whole voxel grid.
         *
         * This method is a callback method for "drawTSDF".
         *
         * @param visualizer control class for the open3d window.
         * @return true in any case.
         */
        bool TSDFDrawer::drawFullView(open3d::visualization::Visualizer *visualizer) {
            visualizer->ClearGeometries();
            visualizer->AddGeometry(tsdfPointer, false);
            return true;
        }

        /**
         * Draw a whole voxel grid in an open3d window and control the display of slices of the grid.
         *
         * The left and right arrow move the slice forward and backward.
         * The down arrow resets the view to the whole grid.
         *
         * @param voxelGridPointer pointer to an open3d's representation of a voxel grid to be displayed.
         */
        void TSDFDrawer::drawTSDF(const std::shared_ptr<open3d::geometry::VoxelGrid> &voxelGridPointer) {
            tsdfPointer = voxelGridPointer;

            std::map<int, std::function<bool(open3d::visualization::Visualizer *)>> myMap;

            std::function<bool(open3d::visualization::Visualizer *)> forwardSlicing =
                    std::bind(&TSDFDrawer::loopForwardThroughSlices, this, std::placeholders::_1);
            myMap.insert(std::make_pair((int) GLFW_KEY_LEFT, forwardSlicing));

            std::function<bool(open3d::visualization::Visualizer *)> backwardSlicing =
                    std::bind(&TSDFDrawer::loopBackwardThroughSlices, this, std::placeholders::_1);
            myMap.insert(std::make_pair((int) GLFW_KEY_RIGHT, backwardSlicing));

            std::function<bool(open3d::visualization::Visualizer *)> orientationChanging =
                    std::bind(&TSDFDrawer::changeOrientation, this, std::placeholders::_1);
            myMap.insert(std::make_pair((int) GLFW_KEY_O, orientationChanging));

            std::function<bool(open3d::visualization::Visualizer *)> fullView =
                    std::bind(&TSDFDrawer::drawFullView, this, std::placeholders::_1);
            myMap.insert(std::make_pair((int) GLFW_KEY_F, fullView));

            open3d::visualization::DrawGeometriesWithKeyCallbacks({tsdfPointer}, myMap);
        }

        /**
             * Save a slice of the already built tsdf as an image.
             * A slice is an object of class VoxelGrid of Open3D, but only consists of voxels with a given z-value.
             * Slices can be viewed as "layers" of the tsdf.
             *
             * The PNGs are built using cairo.
             *
             * The method first creates the slice by iterating through the tsdf.
             * Then, it calculates the size of the slice and initializes the image with a white background.
             * Last, all voxels are painted as 10x10 rectangles with their colors unchanged.
             *
             * @param imageSliceIndex index of the slice to be drawn
             * @param filename where to save the PNG image
             */
        void
        TSDFDrawer::saveSliceAsPNG(const int imageSliceIndex, const int imageSliceOrientation, const char *filename,
                                   const std::shared_ptr<open3d::geometry::VoxelGrid> &voxelGridPointerSlice) {
            // Build slice from full TSDF
            tsdfPointer = voxelGridPointerSlice;

            std::shared_ptr<open3d::geometry::VoxelGrid> slicedVoxelGridPointer =
                    std::make_shared<open3d::geometry::VoxelGrid>(open3d::geometry::VoxelGrid());

            slicedVoxelGridPointer->voxel_size_ = tsdfPointer->voxel_size_;
            slicedVoxelGridPointer->origin_ = tsdfPointer->origin_;

            for (open3d::geometry::Voxel nextVoxel : tsdfPointer->GetVoxels()) {
                if (nextVoxel.grid_index_(imageSliceOrientation) == imageSliceIndex) {
                    slicedVoxelGridPointer->AddVoxel(nextVoxel);
                }
            }

            // Initialize image as cairo surface. It has to be big enough for all voxels in the slice.
            Eigen::Vector3i maxIndices = slicedVoxelGridPointer->GetVoxel(slicedVoxelGridPointer->GetMaxBound());
            Eigen::Vector3i minIndices = slicedVoxelGridPointer->GetVoxel(slicedVoxelGridPointer->GetMinBound());
            int imageWidth;
            int imageHeight;
            switch (imageSliceOrientation) {
                case 0:
                    imageWidth = (maxIndices - minIndices).y();
                    imageHeight = (maxIndices - minIndices).z();
                    break;
                case 1:
                    imageWidth = (maxIndices - minIndices).x();
                    imageHeight = (maxIndices - minIndices).z();
                    break;
                case 2:
                    imageWidth = (maxIndices - minIndices).x();
                    imageHeight = (maxIndices - minIndices).y();
                    break;
                default:
                    return;
            }


            cairo_surface_t *surface;
            surface = cairo_image_surface_create(CAIRO_FORMAT_RGB24, 10 * imageWidth, 10 * imageHeight);
            cairo_t *cr;
            cr = cairo_create(surface);

            // Draw background white
            cairo_rectangle(cr, 0.0, 0.0, 10.0 * imageWidth, 10.0 * imageHeight);
            cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
            cairo_fill(cr);

            // Draw all voxels as rectangles
            for (open3d::geometry::Voxel nextVoxel : slicedVoxelGridPointer->GetVoxels()) {
                double xVoxel = 0.0;
                double yVoxel = 0.0;
                switch(imageSliceOrientation) {
                    case 0:
                        xVoxel = (nextVoxel.grid_index_ - minIndices).y();
                        yVoxel = (nextVoxel.grid_index_ - minIndices).z();
                        break;
                    case 1:
                        xVoxel = (nextVoxel.grid_index_ - minIndices).x();
                        yVoxel = (nextVoxel.grid_index_ - minIndices).z();
                        break;
                    case 2:
                        xVoxel = (nextVoxel.grid_index_ - minIndices).x();
                        yVoxel = (nextVoxel.grid_index_ - minIndices).y();
                        break;
                }

                cairo_rectangle(cr, 10 * xVoxel, 10 * yVoxel, 10.0, 10.0);
                cairo_set_source_rgb(cr, nextVoxel.color_.x(), nextVoxel.color_.y(), nextVoxel.color_.z());
                cairo_fill(cr);
            }
            cairo_surface_write_to_png(surface, filename);

        }
    }  // namespace mapping
}   // namespace cartographer
#endif
