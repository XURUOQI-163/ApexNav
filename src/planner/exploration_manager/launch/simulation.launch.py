"""Select the discrete-action or continuous-velocity simulation planner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def select_planner(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('discrete', 'continuous'):
        raise ValueError('mode must be discrete or continuous')
    filename = 'exploration.launch.py' if mode == 'discrete' else 'exploration_traj.launch.py'
    path = os.path.join(get_package_share_directory('exploration_manager'), 'launch', filename)
    # The upstream "is_real_world" flag chooses the trajectory FSM. In this
    # launch both modes are explicitly wired to Habitat topics, with no hardware.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={
            'is_real_world': 'true' if mode == 'continuous' else 'false',
            'odom_topic': '/habitat/odom',
            'sensor_pose_topic': '/habitat/sensor_pose',
            'depth_topic': '/habitat/camera_depth',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='discrete', choices=['discrete', 'continuous'],
                              description='Must match habitat_evaluation.py --mode'),
        OpaqueFunction(function=select_planner),
    ])
