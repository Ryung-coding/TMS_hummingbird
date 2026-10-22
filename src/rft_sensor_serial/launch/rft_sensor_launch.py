import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_setserial_actions(context, *args, **kwargs):
    # 런치 시점에 dev_names 값을 가져와서 반복 실행
    dev_names = LaunchConfiguration('dev_names').perform(context).split(',')
    actions = []
    for dev in dev_names:
        actions.append(
            ExecuteProcess(
                cmd=['setserial', dev, 'low_latency'],
                shell=True
            )
        )
    return actions

def generate_launch_description():
    package_dir = get_package_share_directory('rft_sensor_serial')
    param_file_path = os.path.join(package_dir, 'config', 'params.yaml')

    # DEV_NAMES 런치 인자로 선언
    declare_dev_names_arg = DeclareLaunchArgument(
        'dev_names',
        default_value='/dev/ttyUSB0',
        description='Comma-separated list of device names'
    )

    return LaunchDescription([
        declare_dev_names_arg,

        # 런치 시점에 setserial 실행
        OpaqueFunction(function=generate_setserial_actions),

        # 파라미터 파일 런치 인자
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file_path,
            description='Path to the ROS2 parameters yaml file to use.'
        ),

        # RFT Sensor Node 실행
        Node(
            package='rft_sensor_serial',
            executable='rft_sensor_serial',
            name='rft_sensor_serial',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
