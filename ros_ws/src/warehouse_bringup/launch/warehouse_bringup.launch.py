import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def _load_parking(map_name):
    cfg_path = f'/configs/{map_name}.yaml'
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    parking = cfg.get('parking', {})
    if not parking:
        robots_cfg = cfg.get('robots', {})
        parking = {k: {'x': v.get('start_x', 0), 'y': v.get('start_y', 0)} for k, v in robots_cfg.items()}
    return sorted(parking.items(), key=lambda item: item[0])


def _resolve_num_robots(context, num_robots_arg):
    num_robots_str = LaunchConfiguration('num_robots').perform(context)
    if num_robots_str:
        try:
            return int(num_robots_str)
        except ValueError:
            pass
    num_robots_env = os.environ.get('NUM_ROBOTS', '')
    if num_robots_env:
        try:
            return int(num_robots_env)
        except ValueError:
            pass
    return None  # means all parking


def launch_setup(context, *args, **kwargs):
    map_name = LaunchConfiguration('map').perform(context)
    if not map_name:
        map_name = EnvironmentVariable('MAP', default_value='simple').perform(context)

    parking_list = _load_parking(map_name)

    num_robots_arg = LaunchConfiguration('num_robots').perform(context)
    resolved = _resolve_num_robots(context, num_robots_arg)
    if resolved is None:
        num_active = len(parking_list)
    else:
        num_active = min(max(resolved, 0), len(parking_list))

    active_robots = parking_list[:num_active]
    all_robots = parking_list

    actions = []

    base_env = os.environ.copy()

    # Robot drivers for active robots only
    for robot_id, pos in active_robots:
        env = dict(base_env)
        env['ROBOT_ID'] = robot_id
        env['MAP'] = map_name
        actions.append(ExecuteProcess(
            cmd=['python3', '/ros_ws/src/robot_sim/robot_sim/robot_driver.py'],
            env=env,
            output='screen',
            name=f'robot_driver_{robot_id}',
        ))

    # Bridge arguments for active robots only
    bridge_args = []
    for robot_id, pos in active_robots:
        bridge_args.extend([
            f'/odom_{robot_id}@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/turtle_{robot_id}/joint/joint_left/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
            f'/model/turtle_{robot_id}/joint/joint_right/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
        ])

    if bridge_args:
        actions.append(TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + bridge_args,
                    env=base_env,
                    output='screen',
                    name='ros_gz_bridge',
                )
            ]
        ))

    return actions


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=EnvironmentVariable('MAP', default_value='simple'),
        description='Map name to load (simple/stress)',
    )

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value=EnvironmentVariable('NUM_ROBOTS', default_value=''),
        description='Number of active robots (default: all parking spots)',
    )

    return LaunchDescription([
        map_arg,
        num_robots_arg,
        OpaqueFunction(function=launch_setup),
    ])
