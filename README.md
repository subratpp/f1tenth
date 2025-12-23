# f1tenth
F1Tenth Race Stack for Autonomous Navigation

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

For foxglove_bridge: need to source galactic: source /opt/ros/galactic/setup.bash
ros2 run foxglove_bridge foxglove_bridge


connect bt: connect_joy on terminal

## Bringup
bringup: ros2 launch f1tenth_stack bringup_launch.py


No Machine:
Check Status: sudo /usr/NX/bin/nxserver --status
Start: sudo /usr/NX/bin/nxserver --startup

# Jetson Clock
sudo /usr/bin/jetson_clocks

# SLAM
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml



## save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rlspeed/race_stack/f1tenth/hesl'}}"


## save posegraph
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/rlspeed/race_stack/f1tenth/hesl'}"

Deserialize pose
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '/home/rlspeed/race_stack/f1tenth/hesl'}"
save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rlspeed/race_stack/f1tenth/hesl'}}"


# Intelisense Camera IMU
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_color:=false \
    enable_depth:=false \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    publish_tf:=true

It gives topic:
/camera/imu
tf: camera_link → camera_imu_frame
Need to create tf_static base_link -> camera_link


# Particle Filter 2017

ros2 launch particle_filter localize_launch.py
rviz map: the map does not show up automatically, change the durability policy to transient (Map -> Topic -> Durability Policy). This is because the map is running on server.


# Cartographer
ros2 launch cartographer cartographer.launch.py

## save map
ros2 run nav2_map_server map_saver_cli -f ~/race_stack/f1tenth/test



# loacalixation
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/rlspeed/race_stack/f1tenth/hesl.yaml}"
or?
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
"{map_url: '/home/rlspeed/race_stack/f1tenth/hesl.yaml'}"




# EKF (odm - imu fusion)
ros2 launch state_estimation ekf.launch.py

=======
# Final
bringup: ros2 launch f1tenth_stack bringup_launch.py
ekf: ros2 launch state_estimation ekf.launch.py

slam: ros2 launch slam_toolbox online_async_launch.py params_file:=/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
save_map: ros2 run nav2_map_server map_saver_cli -f ~/race_stack/f1tenth/test


---
get_traj: obtain it from the notebook
---
pf: ros2 launch particle_filter localize_launch.py
run_pp: ros2 launch pure_pursuit pure_pursuit_launch.py
