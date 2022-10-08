import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    pnc_share_dir = get_package_share_directory("pnc_ros")
    pnc_base_dir = "/".join(pnc_share_dir.split("/")[:-4]) + "/"
    pnc_main_path = pnc_base_dir + os.path.join("simulator", "pybullet", "towr_mpc_main.py")

    launch_pnc_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pnc_share_dir,
                "launch",
                "display.launch.py"
            )
        )
    )

    launch_pnc = ExecuteProcess(
        cmd=["python3",
        pnc_main_path,
        os.path.join(pnc_base_dir, "robot_model", "atlas", "atlas.urdf"),
        os.path.join(pnc_base_dir, "robot_model", "envs", "example_env.urdf")],
        shell=True
    )

    return launch.LaunchDescription([
        launch_pnc_rviz,
        launch_pnc
    ])