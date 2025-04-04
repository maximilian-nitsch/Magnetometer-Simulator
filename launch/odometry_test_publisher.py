# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
# Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
# All rights reserved.

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    topic_name_arg = DeclareLaunchArgument(
        "odom_rate",
        default_value="250.0",
        description="Data rate (frequency) of ground truth odometry",
    )

    # Add the launch argument to the launch description
    ld.add_action(topic_name_arg)
    # Create the node
    odometry_test_publisher_node = Node(
        package="mag_simulator_package",
        namespace="/auv/odometry",
        executable="odometry_test_publisher_node",
        name="odometry_test_publisher_node",
        output="screen",
        parameters=[{"odom_rate": LaunchConfiguration("odom_rate")}],
    )

    ld.add_action(odometry_test_publisher_node)

    return ld
