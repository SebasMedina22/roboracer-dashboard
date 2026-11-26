"""Start everything the dashboard needs. Does NOT start Gazebo.

Run a simulation first (gazebo.launch.py, sim_slam_rviz.launch.py or
wall_follow.launch.py), then:

    ros2 launch roboracer_dashboard dashboard.launch.py

and open http://localhost:8000 in the browser.
"""
from launch import LaunchDescription
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def installed(package):
    try:
        get_package_share_directory(package)
        return True
    except PackageNotFoundError:
        return False


def rosapi_or_warning():
    if not installed('rosapi'):
        return LogInfo(msg='[dashboard] rosapi not installed: automatic topic detection off '
                           '(sudo apt install ros-humble-rosbridge-suite)')
    return Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen')


def video_or_warning():
    # Optional: without it only the camera panel stays empty (e.g. on a car where
    # we cannot install packages).
    if not installed('web_video_server'):
        return LogInfo(msg='[dashboard] web_video_server not installed: camera panel off '
                           '(sudo apt install ros-humble-web-video-server)')
    return Node(package='web_video_server', executable='web_video_server',
                name='web_video_server', parameters=[{'port': 8080}], output='screen')


def generate_launch_description():
    web_port = LaunchConfiguration('web_port')

    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='8000',
                              description='Port of the dashboard web page'),

        # ROS topics <-> browser, as JSON over a WebSocket (ws://<host>:9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),

        # Topic list for the page's automatic topic detection (service /rosapi/topics).
        # Optional: without rosapi the page simply uses the default / URL topic names.
        rosapi_or_warning(),

        # sensor_msgs/Image -> MJPEG video over HTTP (http://<host>:8080); optional
        video_or_warning(),

        # Our page (http://<host>:8000)
        Node(
            package='roboracer_dashboard',
            executable='web_server',
            name='dashboard_web_server',
            parameters=[{'port': ParameterValue(web_port, value_type=int)}],
            output='screen',
        ),
    ])
