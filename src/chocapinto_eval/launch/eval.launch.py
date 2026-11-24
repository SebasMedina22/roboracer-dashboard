"""One evaluated simulation run of the course's cone pipeline.

Starts, without modifying anything in rrstack:
  1. roboracer_planning/cone_drive.launch.py (Gazebo + YOLO + Delaunay + pure pursuit),
     headless by default
  2. a bridge for Gazebo's true poses -> /eval/ground_truth
  3. race_monitor, which writes summary.json / trajectory.csv / events.csv
When race_monitor ends (laps done, contact, stuck or time limit) the whole launch shuts down.

    ros2 launch chocapinto_eval eval.launch.py laps:=3 \
        out_dir:=/mnt/c/Users/Asus/Documents/avai_runs
"""
import os
import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

GROUND_TRUTH_TOPIC = '/eval/ground_truth'


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def _world_name(world_file: str) -> str:
    with open(world_file, encoding='utf-8') as f:
        m = re.search(r'<world\s+name="([^"]+)"', f.read())
    if not m:
        raise RuntimeError(f'no <world name="..."> in {world_file}')
    return m.group(1)


def _monitor_and_bridge(context):
    world_file = LaunchConfiguration('world_file').perform(context)
    if not world_file:
        from ament_index_python.packages import get_package_share_directory
        world_file = os.path.join(get_package_share_directory('roboracer_gazebo'),
                                  'worlds', 'flw_cone_track_from_walls.world')
    gz_topic = f'/world/{_world_name(world_file)}/dynamic_pose/info'

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ground_truth_bridge',
        arguments=[f'{gz_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        remappings=[(gz_topic, GROUND_TRUTH_TOPIC)],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    monitor = Node(
        package='chocapinto_eval', executable='race_monitor', name='race_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': world_file,
            'ground_truth_topic': GROUND_TRUTH_TOPIC,
            'laps': int(LaunchConfiguration('laps').perform(context)),
            'stop_on_contact': _as_bool(LaunchConfiguration('stop_on_contact').perform(context)),
            'max_sim_time': float(LaunchConfiguration('max_sim_time').perform(context)),
            'max_wall_time': float(LaunchConfiguration('max_wall_time').perform(context)),
            'out_dir': LaunchConfiguration('out_dir').perform(context),
            'run_name': LaunchConfiguration('run_name').perform(context),
        }],
    )
    stop_all = RegisterEventHandler(OnProcessExit(
        target_action=monitor,
        on_exit=[EmitEvent(event=Shutdown(reason='race_monitor finished'))],
    ))
    return [bridge, monitor, stop_all]


def generate_launch_description():
    cone_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('roboracer_planning'), 'launch', 'cone_drive.launch.py'])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz window'),
        DeclareLaunchArgument('laps', default_value='3', description='Laps per run'),
        DeclareLaunchArgument('stop_on_contact', default_value='true',
                              description='End the run at the first cone contact'),
        DeclareLaunchArgument('max_sim_time', default_value='900.0',
                              description='Safety stop [s of simulation time]'),
        DeclareLaunchArgument('max_wall_time', default_value='0.0',
                              description='Stop after this many real seconds (0 = no limit)'),
        DeclareLaunchArgument('out_dir', default_value='~/avai_runs',
                              description='Results folder'),
        DeclareLaunchArgument('run_name', default_value='',
                              description='Subfolder name (default: date_time)'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='Cones world (default: roboracer_gazebo cones world)'),
        cone_drive,
        OpaqueFunction(function=_monitor_and_bridge),
    ])
