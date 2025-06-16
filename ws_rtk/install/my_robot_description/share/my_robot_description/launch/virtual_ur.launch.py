from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Виртуальный контроллер UR
        Node(
            package='ur_robot_driver',
            executable='ur_robot_driver_node',
            name='ur_driver',
            parameters=[
                {'robot_ip': '127.0.0.1'},
                {'headless_mode': True},
                {'use_fake_hardware': True}
            ]
        ),
        
        # RViz с конфигом для UR
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '$(find ur_description)/rviz/ur.rviz']
        )
    ])
