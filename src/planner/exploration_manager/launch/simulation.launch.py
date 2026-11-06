"""Select the discrete-action or continuous-velocity simulation planner."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def select_planner(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('discrete', 'continuous'):
        raise ValueError('mode must be discrete or continuous')
    planner = LaunchConfiguration('planner').perform(context).upper()
    utm = LaunchConfiguration('utm').perform(context).lower() == 'true'
    memory_actions = []
    if utm:
        if planner != 'APEXNAV':
            raise ValueError('UTM records the global APEXNAV Skeleton/Frontier graph; use planner:=APEXNAV')
        # Symlink install resolves this file back to the project source tree.
        root = next((p for p in Path(__file__).resolve().parents if (p / 'habitat_evaluation.py').exists()), None)
        if root is None:
            raise RuntimeError('UTM launch requires the ApexNav-ROS2 source project (build with --symlink-install)')
        directory = LaunchConfiguration('utm_directory').perform(context) or str(root / 'data/topology')
        config = os.path.join(get_package_share_directory('exploration_manager'), 'config', 'topology_memory.yaml')
        memory_actions = [
            SetLaunchConfiguration('skeleton', 'true'),
            ExecuteProcess(cmd=[LaunchConfiguration('utm_python'), '-m', 'topology_memory.ros_node',
                                '--ros-args', '--params-file', config, '-p', 'directory:=' + directory],
                           cwd=str(root), output='screen', sigterm_timeout='15'),
        ]
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
    return memory_actions + [
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
        DeclareLaunchArgument('utm', default_value='false', choices=['true', 'false'],
                              description='Persist unified topology; automatically enables skeleton observer'),
        DeclareLaunchArgument('utm_directory', default_value='',
                              description='Empty selects project data/topology; mismatch scenes are archived'),
        DeclareLaunchArgument('utm_python', default_value='/usr/bin/python3',
                              description='Python with ROS2, cv_bridge and OpenCV; no Habitat/model environment needed'),
        DeclareLaunchArgument('topology_sector_degrees', default_value='30.0'),
        DeclareLaunchArgument('topology_max_distance', default_value='5.0'),
        OpaqueFunction(function=select_planner),
    ])
