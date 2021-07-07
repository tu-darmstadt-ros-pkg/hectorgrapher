Cartographer for radar 
============


Purpose
=======

[Cartographer](https://github.com/googlecartographer/cartographer) 
is a system that provides real-time simultaneous localization
and mapping ([SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) ) in 2D and 3D across multiple platforms and sensor
configurations.

The purpose of this branch is to provide optimal localization and mapping for object penetrating sensors with sparse 
data and high noise, like radar.

[![Cartographer Radar SLAM Demo](https://j.gifs.com/3620x9.gif "Cartographer Radar SLAM Demo, TU Darmstadt SIM Lab")](https://youtu.be/kJNA30l0X-w)


# Getting started

You can use our [Cartographer ROS Integration](https://github.com/tu-darmstadt-ros-pkg/cartographer_ros/tree/tsdf_radar-noetic).

Additional Links:
* Learn to use Cartographer at [Cartographer](https://google-cartographer.readthedocs.io) 
* [Fine tuning Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html) 


## Experimental Data

Example ROS Bag files can be downloaded from [here](https://duckduckgo.com), containing amongst other things the 
following data, in both good visual conditions and under smoke:
-  raw and processed radar data
-  LiDAR data as reference
-  IMU & odometry data

### Additional Resources

#### LUA Config
A first set of working parameters are being provided in several config files 
[here](https://github.com/tu-darmstadt-ros-pkg/cartographer_ros/tree/tsdf_radar-noetic/cartographer_ros/configuration_files). 
The same tuning methodology for regular Cartographer may be applied, with some additional notes:
- For the radar sensor, the number of accumulated range data should cover approx. one full rotation (approx. 60 scans)
- Correlative scan matching should be used beforehand, since it provides a good first estimate and is fast for the 
  sparse radar scans
- The voxel filter has to be adapted for the radar, by decreasing the filter size and increasing the min. number of 
  points, in comparison to LiDAR scans
- The accuracy of the SLAM is very sensitive against the following weights, 
  which may vary between different systems and setups:
    - `TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight`
    - `TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.free_space_weight`
    - `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight`
    - `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight`
    - `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight`
    - `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.empty_space_cost`

##### Additional Config Parameters

- `TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space_only_first_hits`
    
    use the radar sensor model by taking multiple detections in range direction into account. 
    
    Only the free space between the sensor and the first detection for a fixed angle is assumed to be free.
  

- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.use_gnc`

    for enabling robust optimization by [Graduated Non Convexity](https://arxiv.org/pdf/1909.08605.pdf).
    
    The following options for `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.<...>` are available:
    - `max_iterations`: Number of maximum iterations
    - `gm_shape`: The shape of the GM function
    - `non_convexity_stop`: The non-convexity of the GM function to finish optimization. 
      Set under 1 to prevent early convergence.
    - `min_convexity`: Overrides the minimum convexity, which is calculated in the first optimization step.
    - `non_convexity_inc_factor`: The update factor for the non convexity: 
      current convexity /= non_convexity_inc_factor
      
<!--- https://gifs.com/gif/radar-slam-3620x9 --->
