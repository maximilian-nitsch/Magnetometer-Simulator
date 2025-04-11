# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

import os
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare the path to the config YAML file
    config_file_path = os.path.join(
        get_package_share_directory('mag_simulator_package'),
        'config',
        'pni_rm3100.yaml',
    )

    # Open the YAML file and load the parameters
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    ld.add_action(use_sim_time_arg)

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/auv/odometry',
        description='Topic name of the ground truth odometry from vehicle',
    )
    ld.add_action(odom_topic_arg)

    # Create the node
    mag_simulator_package_node = Node(
        package='mag_simulator_package',
        namespace='/auv/gnc/navigation_sensors/magnetometer',
        executable='mag_simulator_package_node',
        name='mag_simulator_node',
        output='screen',
        parameters=[
            config,
            {'topic_name': LaunchConfiguration('odom_topic')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    ld.add_action(mag_simulator_package_node)

    return ld