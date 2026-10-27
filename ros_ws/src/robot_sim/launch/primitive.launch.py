from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '/ros_ws/src/warehouse_world/worlds/empty_box.sdf'],
            output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
             output='screen'),
        Node(package='robot_sim', executable='primitive_node', output='screen'),
    ])
