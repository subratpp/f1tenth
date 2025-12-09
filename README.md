# f1tenth
F1Tenth Race Stack for Autonomous Navigation



connect bt: connect_joy on terminal

bringup: ros2 launch f1tenth_stack bringup_launch.py


No Machine:
Check Status: sudo /usr/NX/bin/nxserver --status
Start: sudo /usr/NX/bin/nxserver --startup


# SLAM
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml