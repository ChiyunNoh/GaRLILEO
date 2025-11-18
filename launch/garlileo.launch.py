import os

from launch          import LaunchDescription
from launch.actions   import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    garlileo_share = get_package_share_directory('garlileo')

    default_cfg_path   = os.path.join(garlileo_share, 'dataset', 'GaRLILEO', 'config.yaml')
    default_bag_path   = "/path/to/your/dataset" 

    default_start_time = '0'


    cfg_arg   = DeclareLaunchArgument('config_path', default_value=default_cfg_path)
    bag_arg   = DeclareLaunchArgument('rosbag_path', default_value=default_bag_path)
    start_arg = DeclareLaunchArgument('start_time',  default_value=default_start_time)

    cfg_path   = LaunchConfiguration('config_path')
    bag_path   = LaunchConfiguration('rosbag_path')
    start_time = LaunchConfiguration('start_time')


    garlileo_node = Node(
        package='garlileo',
        executable='garlileo_node_exe',
        name='garlileo_node',
        output='screen',
        parameters=[{
            'config_path': cfg_path
        }]
    )

    spot_msgs_node = Node(
        package='spot_msgs',
        executable='spot_msgs_node',
        name='spot_msgs_node2',
        output='screen'
    )


    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path,
             '--clock',                      
             '--start-offset', start_time],
        output='screen'
    )

    rviz_cfg = PathJoinSubstitution(
        [garlileo_share, 'config', 'rviz.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_cfg]
    )


    return LaunchDescription([
        cfg_arg, bag_arg, start_arg,        
        spot_msgs_node,
        bag_play,
        garlileo_node,
        rviz,
    ])
