# f1tenth
F1Tenth Race Stack for Autonomous Navigation



connect bt: connect_joy on terminal

bringup: ros2 launch f1tenth_stack bringup_launch.py


No Machine:
Check Status: sudo /usr/NX/bin/nxserver --status
Start: sudo /usr/NX/bin/nxserver --startup

# Jetson Clock
sudo /usr/bin/jetson_clocks

# SLAM
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml



## save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rlspeed/race_stack/f1tenth/hes'}}"


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

roslaunch particle_filter localize.launch


# Cartographer
ros2 launch my_cartographer_config cartographer.launch.py

## save map
ros2 run nav2_map_server map_saver_cli -f ~/my_f1tenth_map
