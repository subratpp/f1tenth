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

ros2 launch slam_toolbox online_async_launch.py params_file:=/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml use_sim_time:=false


## save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rlspeed/race_stack/f1tenth/hes'}}"


## save posegraph
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/rlspeed/race_stack/f1tenth/hesl'}"

Deserialize pose
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '/home/rlspeed/race_stack/f1tenth/hesl'}"
save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rlspeed/race_stack/f1tenth/hesl'}}"


