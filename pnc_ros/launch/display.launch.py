import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys

from ament_index_python import get_package_share_directory

def generate_launch_description():
    pnc_share_dir = get_package_share_directory("pnc_ros")
    pnc_base_dir = "/".join(pnc_share_dir.split("/")[:-4]) + "/"
    default_robot_urdf_path = os.path.join(pnc_share_dir, "urdf", "atlas", "atlas.urdf"),
    default_env_urdf_path = os.path.join(pnc_base_dir, "robot_model", "envs", "example_env.urdf")
    default_rviz_config_path = os.path.join(pnc_share_dir, 'rviz/pnc_config.rviz')

    robot_state_publisher = launch_ros.actions.Node(
        name="robot_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('robot_urdf_path')])}]
    )
    cube_robot_state_publisher_node = launch_ros.actions.Node(
        name="environment_state_publisher",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('env_urdf_path')])}],
        remappings=[("robot_description", "environment_description")]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='robot_urdf_path', default_value=default_robot_urdf_path,
                                            description='Absolute path to robot URDF'),
        launch.actions.DeclareLaunchArgument(name='env_urdf_path', default_value=default_env_urdf_path,
                                            description='Absolute path to environment URDF'),
        robot_state_publisher,
        cube_robot_state_publisher_node,
        rviz_node
    ])