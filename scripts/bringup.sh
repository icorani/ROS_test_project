#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

MAP="${MAP:-simple}"
NUM_ROBOTS="${NUM_ROBOTS:-}"
CONFIG="/configs/${MAP}.yaml"

# Determine active robots and bridge topics from config via Python
python3 -c "
import yaml, os
with open('$CONFIG') as f:
    cfg = yaml.safe_load(f)
parking = sorted(cfg.get('parking', {}).items(), key=lambda item: item[0])
all_robots = [f'robot_{i+1}' for i, (pid, pos) in enumerate(parking)]
num_robots_env = os.environ.get('NUM_ROBOTS', '')
if num_robots_env:
    try:
        limit = int(num_robots_env)
    except ValueError:
        limit = len(all_robots)
else:
    limit = len(all_robots)
limit = min(max(limit, 0), len(all_robots))
active = all_robots[:limit]
bridge = []
for rid in active:
    bridge.extend([
        f'/odom_{rid}@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        f'/model/turtle_{rid}/joint/joint_left/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
        f'/model/turtle_{rid}/joint/joint_right/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
    ])
with open('/tmp/active_robots', 'w') as f:
    f.write(' '.join(active))
with open('/tmp/bridge_topics', 'w') as f:
    f.write(' '.join(bridge))
"

ACTIVE_ROBOTS=$(cat /tmp/active_robots)
BRIDGE_TOPICS=$(cat /tmp/bridge_topics)

# Launch robot drivers for active robots only
for rid in $ACTIVE_ROBOTS; do
    ROBOT_ID=$rid MAP=$MAP python3 /ros_ws/src/robot_sim/robot_sim/robot_driver.py &
done

# Launch gazebo bridge after a short delay so gz sim is ready
(
    sleep 5
    if [ -n "$BRIDGE_TOPICS" ]; then
        ros2 run ros_gz_bridge parameter_bridge $BRIDGE_TOPICS
    fi
) &

wait
