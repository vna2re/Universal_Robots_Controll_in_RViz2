from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur10e.urdf')
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states']}],
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_ft_frame',
            arguments=['0', '0', '0.1', '0', '0', '0', 'wrist_3_link', 'ft_frame'],
            output='screen'
        )
    ])

