from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_channel_num', default_value='1'),  # 直接パラメータを定義
        DeclareLaunchArgument('can_bitrate', default_value='500000'),  # 直接パラメータを定義
        DeclareLaunchArgument('can_tx_config_path', default_value='can_tx_config.yaml'),  # 直接パラメータを定義
        DeclareLaunchArgument('can_rx_config_path', default_value='can_rx_config.yaml'),  # 直接パラメータを定義
        Node(
            package='ros_kvaser_can_driver',
            executable='kvaser_can_driver',
            name='kvaser_can_driver',
            output='screen',
            parameters=[
                {'device_channel_num': LaunchConfiguration('device_channel_num')},  # パラメータ1の型を整数に変換
                {'can_bitrate': LaunchConfiguration('can_bitrate')},  # パラメータ2の型を整数に変換
                {'can_tx_config_path': LaunchConfiguration('can_tx_config_path')},  # パラメータ3の型を整数に変換
                {'can_rx_config_path': LaunchConfiguration('can_rx_config_path')},  # パラメータ4の型を整数に変換
            ],
        )
    ])