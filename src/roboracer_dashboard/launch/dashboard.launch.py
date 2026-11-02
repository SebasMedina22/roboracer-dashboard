"""Start everything the dashboard needs. Does NOT start Gazebo.

Run a simulation first (gazebo.launch.py, sim_slam_rviz.launch.py or
wall_follow.launch.py), then:

    ros2 launch roboracer_dashboard dashboard.launch.py

and open http://localhost:8000 in the browser.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

        # sensor_msgs/Image -> MJPEG video over HTTP (http://<host>:8080)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8080}],
            output='screen',
        ),

        # Our page (http://<host>:8000)
        Node(
            package='roboracer_dashboard',
            executable='web_server',
            name='dashboard_web_server',
            parameters=[{'port': ParameterValue(web_port, value_type=int)}],
            output='screen',
        ),
    ])
