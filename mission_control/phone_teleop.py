#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
from datetime import datetime

from flask import Flask, jsonify, request, render_template_string

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ackermann_msgs.msg import AckermannDriveStamped


app = Flask(__name__)

# ===============================
# ---  PROCESS ORCHESTRATION  ---
# ===============================
bringup_proc = None
map_proc = None


def start_launch(cmd):
    return subprocess.Popen(
        cmd,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def stop_launch(proc):
    if proc and proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)


def status_of(proc):
    if proc is None:
        return "stopped"
    rc = proc.poll()
    if rc is None:
        return "running"
    if rc == 0:
        return "stopped"
    return "failed"


def shutdown_handler(signum, frame):
    print("\nShutting down, sending SIGINT to child processes...")

    stop_launch(bringup_proc)
    stop_launch(map_proc)

    # allow processes time to exit gracefully
    threading.Timer(2.0, lambda: os._exit(0)).start()

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)


MAP_PARAM_FILE = "/home/rlspeed/race_stack/f1tenth/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml"



# ===============================
# ---   TELEOP SHARED STATE   ---
# ===============================
lock = threading.Lock()

speed = 0.0
steer = 0.0

MAX_SPEED = 1.0
STEER_MAG = 0.20

HZ = 20.0


# ===============================
# -----------  HTML  ------------
# ===============================

HTML = """
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: sans-serif; text-align: center; }

.toolbar-block {
  font-size: 14px;
  padding: 4px 0 6px 0;
  border-bottom: 2px solid #aaa;
}

.rowline {
  margin-top: 4px;
}

.small-btn {
  padding: 6px 10px;
  font-size: 13px;
}

.status-dot {
  display:inline-block;
  width:12px;
  height:12px;
  border-radius:50%;
}
.running { background:#2ecc71; }
.stopped { background:#e74c3c; }
.failed { background:orange; }

input[type=number]{
  width:90px;
  text-align:center;
  font-size:16px;
}

.dpad-btn{
  border-radius:50%;
  width:85px;
  height:85px;
  font-size:28px;
  margin:6px;
}
.center-btn{
  background:#ff5555;
  color:white;
}
</style>
</head>
<body>

<h2>Robot Teleoperation</h2>

<!-- ========= COMPACT ORCHESTRATOR (2 LINES) ========= -->

<div class="toolbar-block">

  <!-- Bringup line -->
  <div class="rowline">
    <b>Bringup:</b>
    <span id="bringupDot" class="status-dot stopped"></span>
    <span id="bringupText">stopped</span>
    <button class="small-btn" onclick="bringup_start()">Start</button>
    <button class="small-btn" onclick="bringup_stop()">Stop</button>
  </div>

  <!-- Map line -->
  <div class="rowline">
    <b>Map:</b>
    <span id="mapDot" class="status-dot stopped"></span>
    <span id="mapText">stopped</span>
    <button class="small-btn" onclick="map_start()">Start</button>
    <button class="small-btn" onclick="map_stop()">Stop</button>

    &nbsp; | &nbsp;
    <button class="small-btn" onclick="save_map()">Save Map</button>
  </div>

</div>

<!-- ================= TELEOP SECTION ================= -->

<h3>Teleop Limits</h3>
Max speed:
<input id="maxSpeed" type="number" step="0.1" value="1.0">
Max |steer|:
<input id="maxSteer" type="number" step="0.01" value="0.20">
<button onclick="apply_limits()">✔ ACCEPT</button>

<p id="limitView" style="font-weight:bold;"></p>

<h3>Current Command</h3>
<p id="cmdView" style="font-size:18px;">speed=0.00 , steer=0.00</p>

<div>
  <button class="dpad-btn" onclick="send('up')">U</button>
</div>
<div>
  <button class="dpad-btn" onclick="send('left')">L</button>
  <button class="dpad-btn center-btn" onclick="send('stop')">X</button>
  <button class="dpad-btn" onclick="send('right')">R</button>
</div>
<div>
  <button class="dpad-btn" onclick="send('down')">D</button>
</div>

<script>
// --- orchestrator ---
function bringup_start(){ fetch("/bringup/start"); }
function bringup_stop(){ fetch("/bringup/stop"); }
function map_start(){ fetch("/map/start"); }
function map_stop(){ fetch("/map/stop"); }
function save_map(){ fetch("/map/save"); }

// --- teleop ---
function send(cmd){
  fetch("/cmd",{method:"POST",headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:cmd})}).then(update_state);
}
function apply_limits(){
  fetch("/set_limits",{method:"POST",headers:{'Content-Type':'application/json'},
    body:JSON.stringify({
      max_speed:document.getElementById('maxSpeed').value,
      steer_mag:document.getElementById('maxSteer').value
    })}).then(update_state);
}

function update_state(){
 fetch("/state").then(r=>r.json()).then(s=>{

   // limits + command
   document.getElementById('limitView').innerText =
     `Limits: max_speed ${s.max_speed.toFixed(2)} , |steer| ≤ ${s.steer_mag.toFixed(2)}`;

   document.getElementById('cmdView').innerText =
     `speed ${s.speed.toFixed(2)} , steer ${s.steer.toFixed(2)}`;

   // bringup state
   document.getElementById('bringupText').innerText=s.bringup;
   document.getElementById('bringupDot').className="status-dot "+s.bringup;

   // map state
   document.getElementById('mapText').innerText=s.map;
   document.getElementById('mapDot').className="status-dot "+s.map;
 });
}

setInterval(update_state, 800);
</script>

</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)

# ---------- orchestrator endpoints ----------
@app.route("/bringup/start")
def bringup_start():
    global bringup_proc
    if bringup_proc and bringup_proc.poll() is None:
        return "already running"
    bringup_proc=start_launch(["ros2","launch","f1tenth_stack","bringup_launch.py"])
    return "ok"

@app.route("/bringup/stop")
def bringup_stop():
    global bringup_proc
    stop_launch(bringup_proc)
    return "ok"

@app.route("/map/start")
def map_start():
    global map_proc
    if map_proc and map_proc.poll() is None:
        return "already running"

    map_proc = start_launch([
        "ros2", "launch",
        "slam_toolbox", "online_async_launch.py",
        f"params_file:={MAP_PARAM_FILE}"
    ])
    return "ok"


@app.route("/map/stop")
def map_stop():
    global map_proc
    stop_launch(map_proc)
    return "ok"

@app.route("/map/save")
def map_save():
    now=datetime.now().strftime("%Y%m%d_%H%M%S")
    name=f"/home/rlspeed/race_stack/f1tenth/map_{now}"
    subprocess.Popen(["ros2","run","nav2_map_server","map_saver_cli","-f",name])
    return jsonify(msg=f"saving map to {name}")

# ---------- shared status ----------
@app.route("/state")
def state():
    with lock:
        return jsonify(
            bringup=status_of(bringup_proc),
            map=status_of(map_proc),
            speed=speed,
            steer=steer,
            max_speed=MAX_SPEED,
            steer_mag=STEER_MAG
        )

# ---------- teleop config ----------
@app.route("/set_limits",methods=["POST"])
def set_limits():
    global MAX_SPEED,STEER_MAG
    d=request.get_json()
    with lock:
        MAX_SPEED=abs(float(d["max_speed"]))
        STEER_MAG=abs(float(d["steer_mag"]))
    return ("",204)

@app.route("/cmd",methods=["POST"])
def cmd():
    global speed,steer
    c=request.get_json()["cmd"]
    with lock:
        if c=="up": speed+=MAX_SPEED
        elif c=="down": speed-=MAX_SPEED
        elif c=="left": steer+=STEER_MAG
        elif c=="right": steer-=STEER_MAG
        elif c=="stop":
            speed=0.0; steer=0.0
        speed=max(-MAX_SPEED,min(MAX_SPEED,speed))
        steer=max(-STEER_MAG,min(STEER_MAG,steer))
    return ("",204)

# ===============================
# ROS2 TELEOP NODE
# ===============================
class DrivePublisher(Node):
    def __init__(self):
        super().__init__("web_teleop")
        self.pub=self.create_publisher(
            AckermannDriveStamped,"/drive",qos_profile_system_default)
        self.create_timer(1.0/HZ,self.on_timer)

    def on_timer(self):
        with lock:
            v=float(speed); s=float(steer)
        msg=AckermannDriveStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.drive.speed=v
        msg.drive.steering_angle=s
        self.pub.publish(msg)

# ===============================
# MAIN
# ===============================
def main():
    rclpy.init()
    node = DrivePublisher()

    # register shutdown handler for Ctrl+C and kill
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    web = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    web.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received")

    finally:
        print("Stopping ROS node and child processes...")

        # stop teleop ROS node
        node.destroy_node()
        rclpy.shutdown()

        # stop launched bringup / mapping
        stop_launch(bringup_proc)
        stop_launch(map_proc)

        print("Clean shutdown complete.")


if __name__=="__main__":
    main()
