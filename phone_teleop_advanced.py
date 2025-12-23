#!/usr/bin/env python3

import threading
import subprocess
import signal
import time
import os
import io

import numpy as np
from PIL import Image
from flask import Flask, request, render_template_string, jsonify, send_file

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


# =========================
# USER CONFIG
# =========================
BRINGUP_CMD = ["ros2", "launch", "f1tenth_bringup", "bringup.launch.py"]
SLAM_CMD    = ["ros2", "launch", "f1tenth_slam", "slam.launch.py"]
SAVE_MAP_CMD = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f",
                os.path.expanduser("~/map")]

MAX_SPEED = 1.0
STEER_MAG = 0.20
PUBLISH_HZ = 10.0
DEADMAN_TIMEOUT = 1.0
MAP_REFRESH_MS = 2000

# =========================
# SHARED STATE
# =========================
speed = 0.0
steer = 0.0
last_cmd_time = time.time()

map_image = None
cmd_lock = threading.Lock()
map_lock = threading.Lock()

processes = {"bringup": None, "slam": None}

# =========================
# FLASK APP
# =========================
app = Flask(__name__)

HTML = """
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: sans-serif; margin:0; text-align:center; }
h2 { margin:8px; }

.tabbtn {
  width: 45%;
  height: 42px;
  font-size: 16px;
  margin: 4px;
}

.panel { margin: 10px 0; }

.value {
  font-weight: bold;
  margin-bottom: 6px;
}

.slider {
  width: 90%;
}

.dpad {
  display: grid;
  grid-template-columns: 70px 70px 70px;
  grid-template-rows: 70px 70px 70px;
  justify-content: center;
  margin: auto;
}

.btn {
  width: 64px;
  height: 64px;
  border-radius: 50%;
  font-size: 22px;
  border: none;
  background: #e0e0e0;
}

.btn:active { background: #bdbdbd; }

.stop {
  background: #e53935;
  color: white;
}

.sys {
  width: 160px;
  height: 44px;
  font-size: 16px;
  margin: 6px;
  border-radius: 10px;
  border: none;
}

.on { background: #4CAF50; color:white; }
.off { background: #f44336; color:white; }
</style>
</head>

<body>

<h2>F1TENTH Control</h2>

<!-- TOP MODE TOGGLE -->
<div>
  <button class="tabbtn" onclick="show('system')">SYSTEM</button>
  <button class="tabbtn" onclick="show('map')">MAP</button>
</div>

<!-- SYSTEM PANEL -->
<div id="system" class="panel">
  <button id="bringup" class="sys off" onclick="toggle('bringup')">BRINGUP</button><br>
  <button id="slam" class="sys off" onclick="toggle('slam')">SLAM</button><br>
  <button class="sys" onclick="saveMap()">SAVE MAP</button>
</div>

<!-- MAP PANEL -->
<div id="map" class="panel" style="display:none">
  <img id="mapimg" src="/map.png" style="width:95%">
</div>

<hr>

<!-- TELEOP CONTROLS -->
<div class="panel">

  <div class="value" id="speedVal">Speed: 0.0</div>
  <input class="slider" type="range" min="-1" max="1" step="1"
         oninput="setSpeed(this.value)">

  <div class="value" id="steerVal">Steer: 0.0</div>
  <input class="slider" type="range" min="-0.36" max="0.36" step="0.05"
         oninput="setSteer(this.value)">

</div>

<div class="panel">
  <div class="dpad">
    <div></div>
    <button class="btn" onclick="cmd('up')">⬆</button>
    <div></div>

    <button class="btn" onclick="cmd('left')">⬅</button>
    <button class="btn stop" onclick="cmd('stop')">⏹</button>
    <button class="btn" onclick="cmd('right')">➡</button>

    <div></div>
    <button class="btn" onclick="cmd('down')">⬇</button>
    <div></div>
  </div>
</div>

<script>
function cmd(c) {
  fetch('/cmd', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:c})});
}

function setSpeed(v) {
  document.getElementById('speedVal').innerText = "Speed: " + v;
  fetch('/cmd', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:'speed', value:parseFloat(v)})});
}

function setSteer(v) {
  document.getElementById('steerVal').innerText = "Steer: " + v;
  fetch('/cmd', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:'steer', value:parseFloat(v)})});
}

function toggle(name) {
  fetch('/toggle', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({name:name})}).then(updateState);
}

function saveMap() {
  fetch('/save_map', {method:'POST'});
}

function updateState() {
  fetch('/state').then(r=>r.json()).then(s=>{
    ['bringup','slam'].forEach(k=>{
      const b = document.getElementById(k);
      if (s[k]) { b.className='sys on'; b.innerText=k.toUpperCase()+' ON'; }
      else { b.className='sys off'; b.innerText=k.toUpperCase()+' OFF'; }
    });
  });
}

function show(name) {
  document.getElementById('system').style.display =
    (name==='system')?'block':'none';
  document.getElementById('map').style.display =
    (name==='map')?'block':'none';
}

setInterval(() => {
  const img = document.getElementById('mapimg');
  if (img && img.style.display !== 'none') {
    img.src = '/map.png?' + Date.now();
  }
}, """ + str(MAP_REFRESH_MS) + """);

setInterval(updateState, 1000);
</script>

</body>
</html>
"""

# =========================
# FLASK ROUTES
# =========================
@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/state')
def state():
    return jsonify(
        bringup=processes["bringup"] is not None,
        slam=processes["slam"] is not None
    )

@app.route('/cmd', methods=['POST'])
def cmd():
    global speed, steer, last_cmd_time
    data = request.get_json()
    with cmd_lock:
        last_cmd_time = time.time()
        if data['cmd'] == "up": speed += 1
        elif data['cmd'] == "down": speed -= 1
        elif data['cmd'] == "left": steer += STEER_MAG
        elif data['cmd'] == "right": steer -= STEER_MAG
        elif data['cmd'] == "stop": speed = steer = 0
        elif data['cmd'] == "speed": speed = float(data['value'])
        elif data['cmd'] == "steer": steer = float(data['value'])

        speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
        steer = max(-STEER_MAG, min(STEER_MAG, steer))
    return ('',204)

@app.route('/toggle', methods=['POST'])
def toggle():
    name = request.get_json()['name']
    if processes[name] is None:
        cmd = BRINGUP_CMD if name=="bringup" else SLAM_CMD
        processes[name] = subprocess.Popen(cmd, preexec_fn=os.setsid)
    else:
        os.killpg(os.getpgid(processes[name].pid), signal.SIGINT)
        processes[name] = None
    return ('',204)

@app.route('/save_map', methods=['POST'])
def save_map():
    subprocess.Popen(SAVE_MAP_CMD)
    return ('',204)

@app.route('/map.png')
def map_png():
    with map_lock:
        if map_image is None:
            return ('',204)
        buf = io.BytesIO()
        map_image.save(buf, format='PNG')
        buf.seek(0)
        return send_file(buf, mimetype='image/png')

# =========================
# ROS NODE
# =========================
class PhoneControl(Node):
    def __init__(self):
        super().__init__('phone_control')
        self.pub = self.create_publisher(
            AckermannDriveStamped, '/drive', qos_profile_system_default)
        # self.sub = self.create_subscription(
        #     OccupancyGrid, '/map', self.map_cb, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            map_qos
        )


    def map_cb(self, msg):
        global map_image
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data).reshape((h,w))
        img = np.zeros((h,w,3), dtype=np.uint8)
        img[data==0] = [255,255,255]
        img[data==100] = [0,0,0]
        img[data==-1] = [200,200,200]
        img = np.flipud(img)
        with map_lock:
            map_image = Image.fromarray(img)

    def run(self):
        rate = self.create_rate(PUBLISH_HZ)
        while rclpy.ok():
            with cmd_lock:
                if time.time() - last_cmd_time > DEADMAN_TIMEOUT:
                    v, s = 0.0, 0.0
                else:
                    v, s = speed, steer

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = v
            msg.drive.steering_angle = s
            self.pub.publish(msg)
            rate.sleep()

# =========================
# MAIN
# =========================
def main():
    rclpy.init()
    node = PhoneControl()

    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False),
        daemon=True
    ).start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        for p in processes.values():
            if p:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
