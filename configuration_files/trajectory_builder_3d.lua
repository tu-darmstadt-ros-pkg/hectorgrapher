-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

MAX_3D_RANGE = 60.

TRAJECTORY_BUILDER_3D = {
  min_range = 1.,
  max_range = MAX_3D_RANGE,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.15,

  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_RANGE,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 1.,
    occupied_space_weight_1 = 6.,
    translation_weight = 5.,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.004,
  },

  imu_gravity_time_constant = 10.,
  rotational_histogram_size = 120,

  submaps = {
    high_resolution = 0.10,
    high_resolution_max_range = 20.,
    low_resolution = 0.45,
    num_range_data = 160,
    grid_type = "PROBABILITY_GRID",
    high_resolution_range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_3D",
      probability_grid_range_data_inserter = {
        hit_probability = 0.55,
        miss_probability = 0.49,
        num_free_space_voxels = 2,
      },
      tsdf_range_data_inserter = {
        relative_truncation_distance = 2.5,
        maximum_weight = 1000.,
        num_free_space_voxels = 0,
        project_sdf_distance_to_scan_normal = false,
        weight_function_epsilon = 1.0,
        weight_function_sigma = 4.,
        normal_estimate_max_nn = 30.,
        normal_estimate_radius = 0.4,
        normal_computation_method = "CLOUD_STRUCTURE",
        min_range = 0.4,
        max_range = 15.0,
        insertion_ratio = 1.0,
        normal_computation_horizontal_stride = 5,
        normal_computation_vertical_stride = 1,
      },
    },
    low_resolution_range_data_inserter = {
        range_data_inserter_type = "PROBABILITY_GRID_INSERTER_3D",
        probability_grid_range_data_inserter = {
            hit_probability = 0.55,
            miss_probability = 0.49,
            num_free_space_voxels = 2,
        },
        tsdf_range_data_inserter = {
            relative_truncation_distance = 2.5,
            maximum_weight = 1000.,
            num_free_space_voxels = 0,
            project_sdf_distance_to_scan_normal = false,
            weight_function_epsilon = 1.0,
            weight_function_sigma = 4.,
            normal_estimate_max_nn = 30.,
            normal_estimate_radius = 0.4,
            normal_computation_method = "CLOUD_STRUCTURE",
            min_range = 1.0,
            max_range = 60.0,
            insertion_ratio = 0.1,
            normal_computation_horizontal_stride = 20,
            normal_computation_vertical_stride = 4,
        },
    },
  },
  optimizing_local_trajectory_builder = {
      high_resolution_grid_weight = 1,
      low_resolution_grid_weight = 1,
      velocity_weight = 1,
      translation_weight = 1,
      rotation_weight = 1,
      odometry_translation_weight = 1,
      odometry_rotation_weight = 1,
      initialize_map_orientation_with_imu = true,
      calibrate_imu = false,
      ct_window_horizon = 0.9,
      ct_window_rate = 0.1,
      imu_integrator = "RK4",
      imu_cost_term = "PREINTEGRATION",
      initialization_duration = 3.0,
      use_adaptive_odometry_weights = true,
      use_per_point_unwarping = false,
      use_multi_resolution_matching = false,
      num_points_per_subdivision = 4,
      control_point_sampling = "CONSTANT",
      sampling_max_delta_translation = 0.2;
      sampling_max_delta_rotation = 0.1;
      sampling_min_delta_time = 0.025;
      sampling_max_delta_time = 0.25;
      velocity_in_state = true,
      odometry_translation_normalization = 2.0e-2;
      odometry_rotation_normalization = 1.0e-1;
  },
}
