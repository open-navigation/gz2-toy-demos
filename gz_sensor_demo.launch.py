# Copyright 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    dir = os.getcwd()
    world = dir + '/world.sdf'
    rviz_config = dir + '/demo.rviz'

    # Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + world
        }.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config],
    )

    # Bridge, but can be put into a yaml file and loaded instead
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar_planar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar_planar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/lidar_3d@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar_3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    # Specialized image transport-powered bridge
    # There's also a specialized bridge for pointclouds
    # image_bridge = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['camera', 'depth_camera', 'depth_camera/image', 'depth_camera/depth_image'],
    #     output='screen'
    # )

    # Minimizing TF, so just setting sensor frames to global frame for visualization
    static_transform_one = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'global_frame', 'invisible_robot/invisible_robot_link/gpu_planar_lidar'],
    )

    static_transform_two = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'global_frame', 'invisible_robot/invisible_robot_link/gpu_lidar'],
    )

    static_transform_three = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'global_frame', 'invisible_robot/invisible_robot_link/rgbd_camera'],
    )
    
    return LaunchDescription([
        gz_sim,
        bridge,
        rviz,
        static_transform_one,
        static_transform_two,
        static_transform_three
    ])
