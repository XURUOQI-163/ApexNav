"""Select the discrete-action or continuous-velocity simulation planner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def select_planner(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('discrete', 'continuous'):
        raise ValueError('mode must be discrete or continuous')
    planner = LaunchConfiguration('planner').perform(context).upper()
    if planner == 'TOPOLOGY':
        if LaunchConfiguration('skeleton').perform(context).lower() == 'true':
            raise ValueError('Skeleton Graph observes the global APEXNAV map; use planner:=APEXNAV')
        if mode != 'continuous':
            raise ValueError('TOPOLOGY requires mode:=continuous (MPC v/w execution)')
        path = os.path.join(get_package_share_directory('topology_exploration'), 'launch', 'topology.launch.py')
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]
    filename = 'exploration.launch.py' if mode == 'discrete' else 'exploration_traj.launch.py'
    path = os.path.join(get_package_share_directory('exploration_manager'), 'launch', filename)
    # The upstream "is_real_world" flag chooses the trajectory FSM. In this
    # launch both modes are explicitly wired to Habitat topics, with no hardware.
    return [
        SetParameter(name='sparse_topology.enabled', value=LaunchConfiguration('topology').perform(context) == 'sparse'),
        SetParameter(name='sparse_topology.sector_degrees', value=float(LaunchConfiguration('topology_sector_degrees').perform(context))),
        SetParameter(name='sparse_topology.max_distance', value=float(LaunchConfiguration('topology_max_distance').perform(context))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={
            'is_real_world': 'true' if mode == 'continuous' else 'false',
            'odom_topic': '/habitat/odom',
            'sensor_pose_topic': '/habitat/sensor_pose',
            'depth_topic': '/habitat/camera_depth',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='APEXNAV', choices=['APEXNAV', 'TOPOLOGY'],
                              description='Grid-based ApexNav or geometric topology with rolling local map'),
        DeclareLaunchArgument('mode', default_value='discrete', choices=['discrete', 'continuous'],
                              description='Must match habitat_evaluation.py --mode'),
        DeclareLaunchArgument('topology', default_value='none', choices=['none', 'sparse'],
                              description='Optional sparse geometry observer; grid planner still controls motion'),
        DeclareLaunchArgument('skeleton', default_value='false', choices=['true', 'false'],
                              description='Independent read-only Skeleton Graph of the global APEXNAV map'),
        DeclareLaunchArgument('topology_sector_degrees', default_value='30.0'),
        DeclareLaunchArgument('topology_max_distance', default_value='5.0'),
        OpaqueFunction(function=select_planner),
    ])
