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

include "map_builder.lua"
include "trajectory_builder.lua"
-- @note 注释参考链接: https://www.yii666.com/blog/601632.html
options = {
  map_builder = MAP_BUILDER,                    -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,      -- trajectory_builder.lua的配置信息
  map_frame = "map",                            -- 地图坐标系的名字
  tracking_frame = "base_link",                 -- 将所有传感器数据转换到这个坐标系下默认imu_link
  published_frame = "base_link",                -- tf: map -> odom 默认"footprint"
  odom_frame = "odom",                          -- 里程计的坐标系名字
  provide_odom_frame = true,                    -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
  publish_frame_projected_to_2d = false,        -- 是否将坐标系投影到平面上
  use_pose_extrapolator = true,                 -- 发布tf时是否使用pose_extrapolator的结果
  use_odometry = false,                         -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                          -- 是否使用gps
  use_landmarks = false,                        -- 是否使用landmark
  num_laser_scans = 0,                          -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 1,               -- 是否使用multi_echo_laser_scans数据
  -- 这两个还有下面的是否使用点云数据不能同时为0
  num_subdivisions_per_laser_scan = 10,         -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                         -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,           -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,              -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,               -- 发布pose的时间间隔，比如：5e-3频率是200Hz
  trajectory_publish_period_sec = 30e-3,        -- 发布轨迹标记的间隔
  rangefinder_sampling_ratio = 1.,              -- 传感器数据的采样频率，多少次数据采样一次，默认都是1
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10   -- 其含义应该应该是多少帧插入一次子图，算法中还有一个乘二操作

return options
