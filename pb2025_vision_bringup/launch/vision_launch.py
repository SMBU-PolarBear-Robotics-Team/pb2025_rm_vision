# Copyright 2025 Lihan Chen
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_vision_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_hik_camera = LaunchConfiguration("use_hik_camera")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "params", "simulation", "vision_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_hik_camera_cmd = DeclareLaunchArgument(
        "use_hik_camera",
        default_value="True",
        description="Whether to bringup hik camera node",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="rm_vision_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                condition=IfCondition(use_hik_camera),
                package="hik_camera_ros2_driver",
                executable="hik_camera_ros2_driver_node",
                name="hik_camera_ros2_driver",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            # Node(
            #     package="armor_detector",
            #     executable="armor_detector_node",
            #     name="armor_detector",
            #     output="screen",
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=["--ros-args", "--log-level", log_level],
            # ),
            Node(
                package="armor_detector_openvino",
                executable="armor_detector_openvino_node",
                name="armor_detector_openvino",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="armor_tracker",
                executable="armor_tracker_node",
                name="armor_tracker",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="projectile_motion",
                executable="projectile_motion_node",
                name="projectile_motion",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                # BUG: Condition not works due to https://github.com/ros2/launch_ros/pull/339#issuecomment-2475743824
                condition=IfCondition(use_hik_camera),
                package="hik_camera_ros2_driver",
                plugin="hik_camera_ros2_driver::HikCameraRos2DriverNode",
                name="hik_camera_ros2_driver",
                parameters=[configured_params],
            ),
            # ComposableNode(
            #     package="armor_detector",
            #     plugin="rm_auto_aim::ArmorDetectorNode",
            #     name="armor_detector",
            #     parameters=[configured_params],
            # ),
            ComposableNode(
                package="armor_detector_openvino",
                plugin="rm_auto_aim::ArmorDetectorOpenVinoNode",
                name="armor_detector_openvino",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="armor_tracker",
                plugin="rm_auto_aim::ArmorTrackerNode",
                name="armor_tracker",
                parameters=[configured_params],
            ),
            ComposableNode(
                package="projectile_motion",
                plugin="projectile_motion::ProjectileMotionNode",
                name="projectile_motion",
                parameters=[configured_params],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_hik_camera_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the vision nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
