import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    dxl_port = LaunchConfiguration('dxl_port').perform(context)
    ft_port = LaunchConfiguration('ft_port').perform(context)
    if os.path.realpath(dxl_port) == os.path.realpath(ft_port):
        raise RuntimeError('Dynamixel and F/T sensor must use different USB ports')
    sensor_params = os.path.join(get_package_share_directory('rft_sensor_serial'), 'config', 'params.yaml')
    actions = []
    if os.path.exists(ft_port):
        actions.append(ExecuteProcess(cmd=['setserial', ft_port, 'low_latency'], output='screen'))
    actions.extend([
        Node(package='rft_sensor_serial', executable='rft_sensor_serial', output='screen',
             parameters=[sensor_params, {'DEV_NAMES': [ft_port]}]),
        Node(package='tms_hummingbird', executable='bldc_control', output='screen'),
        Node(package='tms_hummingbird', executable='dynamixel_control', output='screen',
             parameters=[{'dxl_port': dxl_port}]),
        Node(package='tms_hummingbird', executable='tms_gui.py', output='screen'),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dxl_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('ft_port', default_value='/dev/ttyUSB0'),
        OpaqueFunction(function=create_nodes),
    ])
