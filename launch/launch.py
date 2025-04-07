# Copyright 2024 Intelligent Robotics Lab
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
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('p7')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    target = Node(package= 'p7',
                    executable= 'hsv_filter_main',
                    output= 'screen',
                    parameters= [param_file],
                    remappings= [
                        ('input_image', '/rgbd_camera/image'),
                        ('camera_info', '/rgbd_camera/camera_info'),
                        ('output_detection_2d', '/detection_2d')
    ])

    convert_2d_3d = Node(package='p7',
                            executable= 'detection_2d_to_3d_pc2_main',
                            output='screen',
                            parameters=[param_file],
                            remappings=[
                              # ('input_depth', '/depth_raw/image'),
                              ('input_pointcloud', '/rgbd_camera/points'),
                              ('input_detection_2d', '/detection_2d'),
                              ('camera_info', '/rgbd_camera/camera_info'),
                              ('output_detection_3d', '/detection_3d')
    ])

    obstacle = Node(package= 'p7',
                        executable= 'obstacle_detector_main',
                        output= 'screen',
                        parameters= [param_file],
                        remappings= [
                            ('input_scan', '/scan_raw')
    ])

    avoidance = Node(package='p7',
                        executable= 'control_node_main',
                        output='screen',
                        parameters=[{'use_sim_time': True}],
                        remappings=[
                        ('input_scan', '/scan_raw'),
                        ('output_vel', '/cmd_vel'),
                        ('attractive_vector', '/attractive_vector'), # Vectores publicados
                        ('repulsive_vector', '/repulsive_vector')
    ])

    ld = LaunchDescription()
    ld.add_action(target)
    ld.add_action(convert_2d_3d)
    ld.add_action(obstacle)
    ld.add_action(avoidance)

    return ld
