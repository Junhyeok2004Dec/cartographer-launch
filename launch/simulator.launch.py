from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    ## ***** File paths *****
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')

    config_dir = '/home/user/sim_ws/src/cartographer_ros/config'
    config_file = 'backpack_2d.lua'

    urdf_dir = '/home/user/sim_ws/src/cartographer_ros/urdf'
    urdf_file = os.path.join(urdf_dir, 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            
        output='screen'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=[
            '-configuration_directory', config_dir,  # 상위 폴더의 config 디렉토리 참조
            '-configuration_basename', config_file
        ],
        remappings=[
            ("odom", "/ego_racecar/odom"),
            ('echoes', 'horizontal_laser_2d'),
            ('/map', '/cartomap')  # 리매핑 추가
        ],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05},

        ],
        remappings=[
            ("odom", "/ego_racecar/odom"),
            ('/map', '/cartomap')  # 리매핑 추가
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
