#!/usr/bin/env python3
import yaml
import sys
import os

MAP_NAME = sys.argv[1] if len(sys.argv) > 1 else "simple"
CFG_PATH = f"config_nodes_edges/{MAP_NAME}.yaml"
OUT_PATH = f"gazebo/warehouse_{MAP_NAME}.sdf"

with open(CFG_PATH) as f:
    cfg = yaml.safe_load(f)

b = cfg["bounds"]
x_min, x_max = b["x_min"], b["x_max"]
y_min, y_max = b["y_min"], b["y_max"]
t = b.get("wall_thickness", 0.2)

def wall_box(name, x, y, w, h, z=0.5, height=1.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{w} {h} {height}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{w} {h} {height}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>"""

walls = [
    wall_box("wall_bottom", (x_min+x_max)/2, y_min, x_max-x_min, t),
    wall_box("wall_top",    (x_min+x_max)/2, y_max, x_max-x_min, t),
    wall_box("wall_left",   x_min, (y_min+y_max)/2, t, y_max-y_min),
    wall_box("wall_right",  x_max, (y_min+y_max)/2, t, y_max-y_min),
]

def robot_model(robot_id, start_x, start_y, start_yaw):
    """Генерирует модель робота с уникальными топиками."""
    return f"""
    <model name="turtle_{robot_id}">
      <pose>{start_x} {start_y} 0.1 0 0 {start_yaw}</pose>
      <link name="chassis">
        <inertial><mass>15.0</mass><inertia><ixx>0.1</ixx><iyy>0.15</iyy><izz>0.2</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.3 0.2 0.1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.3 0.2 0.1</size></box></geometry><material><ambient>0.2 0.5 0.8 1</ambient></material></visual>
      </link>
      <link name="wheel_left">
        <pose>0 0.11 -0.05 -1.5707 0 0</pose>
        <inertial><mass>0.1</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.05</radius><length>0.02</length></cylinder></geometry>
          <surface><friction><ode><mu>100.0</mu><mu2>100.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual"><geometry><cylinder><radius>0.05</radius><length>0.02</length></cylinder></geometry></visual>
      </link>
      <link name="wheel_right">
        <pose>0 -0.11 -0.05 -1.5707 0 0</pose>
        <inertial><mass>0.1</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
        <collision name="collision">
          <geometry><cylinder><radius>0.05</radius><length>0.02</length></cylinder></geometry>
          <surface><friction><ode><mu>100.0</mu><mu2>100.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual"><geometry><cylinder><radius>0.05</radius><length>0.02</length></cylinder></geometry></visual>
      </link>
      <link name="caster">
        <pose>-0.1 0 -0.09 0 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.02</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.02</radius></sphere></geometry></visual>
      </link>
      <joint name="joint_left" type="revolute">
        <parent>chassis</parent><child>wheel_left</child><axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="joint_right" type="revolute">
        <parent>chassis</parent><child>wheel_right</child><axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="joint_caster" type="fixed">
        <parent>chassis</parent><child>caster</child>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>joint_left</joint_name>
        <use_force_commands>true</use_force_commands>
        <p_gain>1.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.0</d_gain>
        <cmd_max>2.0</cmd_max>
        <cmd_min>-2.0</cmd_min>
        <topic>/model/turtle_{robot_id}/joint/joint_left/cmd_vel</topic>
      </plugin>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>joint_right</joint_name>
        <use_force_commands>true</use_force_commands>
        <p_gain>1.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>0.0</d_gain>
        <cmd_max>2.0</cmd_max>
        <cmd_min>-2.0</cmd_min>
        <topic>/model/turtle_{robot_id}/joint/joint_right/cmd_vel</topic>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_topic>/odom_{robot_id}</odom_topic>
        <odom_frame>odom</odom_frame>
        <robot_base_frame>chassis</robot_base_frame>
        <dimensions>2</dimensions>
        <odom_publish_frequency>50</odom_publish_frequency>
      </plugin>
    </model>"""

robots_sdf = ""
for robot_id, r in cfg.get("robots", {}).items():
    robots_sdf += robot_model(
        robot_id,
        r.get("start_x", 0),
        r.get("start_y", 0),
        r.get("start_yaw", 0),
    )

sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="empty">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></visual>
      </link>
    </model>
    {''.join(walls)}
    {robots_sdf}
  </world>
</sdf>
"""

os.makedirs("gazebo", exist_ok=True)
with open(OUT_PATH, "w") as f:
    f.write(sdf)
print(f"Generated {OUT_PATH} with {len(cfg.get('robots', {}))} robots")