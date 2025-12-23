#!/usr/bin/env python3

import threading
import time

from flask import Flask, request, render_template_string, jsonify

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ackermann_msgs.msg import AckermannDriveStamped

# =====================
# Shared control state
# =====================
speed = 0.0
steer = 0.0

MAX_SPEED = 1.0
STEER_MAG = 0.20

PUBLISH_HZ = 10.0
lock = threading.Lock()

# =====================
# Flask app
# =====================
app = Flask(__name__)

HTML = """
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; }
    button { width: 90px; height: 55px; font-size: 18px; margin: 6px; }
    input { width: 80px; font-size: 16px; }
    .row { margin: 8px; }
    .stop { background: #ff5555; color: white; }
  </style>
</head>
<body>

<h2>F1TENTH Phone Teleop</h2>

<div class="row">
  Speed:
  <input id="speedVal" type="number" step="0.1" value="1.0">
  <input id="speedSlider" type="range" min="-1" max="1" step="0.1" value="1.0"
         oninput="syncSpeed(this.value)">
  <button onclick="applySpeed()">Apply</button>
</div>

<div class="row">
  Steering:
  <input id="steerVal" type="number" step="0.01" value="0.20">
  <input id="steerSlider" type="range" min="-0.36" max="0.36" step="0.01" value="0.20"
         oninput="syncSteer(this.value)">
  <button onclick="applySteer()">Apply</button>
</div>

<div class="row">
  <b id="current">Current: Speed = 0.0 | Steer = 0.0</b>
</div>

<hr>

<div class="row">
  <button onclick="send('up')">⬆ UP</button>
</div>
<div class="row">
  <button onclick="send('left')">⬅ LEFT</button>
  <button class="stop" onclick="send('stop')">⏹ STOP</button>
  <button onclick="send('right')">RIGHT ➡</button>
</div>
<div class="row">
  <button onclick="send('down')">⬇ DOWN</button>
</div>

<script>
function send(cmd) {
  fetch('/cmd', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({cmd: cmd})
  }).then(updateState);
}

function applySpeed() {
  fetch('/set_speed', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({value: document.getElementById('speedVal').value})
  }).then(updateState);
}

function applySteer() {
  fetch('/set_steer', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({value: document.getElementById('steerVal').value})
  }).then(updateState);
}

function syncSpeed(v) {
  document.getElementById('speedVal').value = v;
}

function syncSteer(v) {
  document.getElementById('steerVal').value = v;
}

function updateState() {
  fetch('/state')
    .then(r => r.json())
    .then(s => {
      document.getElementById('current').innerText =
        `Current: Speed = ${s.speed.toFixed(2)} | Steer = ${s.steer.toFixed(2)}`;
    });
}
</script>

</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/state')
def state():
    with lock:
        return jsonify(speed=speed, steer=steer)

@app.route('/cmd', methods=['POST'])
def cmd():
    global speed, steer
    c = request.get_json()['cmd']

    with lock:
        if c == 'up':
            speed += 1.0
        elif c == 'down':
            speed -= 1.0
        elif c == 'left':
            steer += STEER_MAG
        elif c == 'right':
            steer -= STEER_MAG
        elif c == 'stop':
            speed = 0.0
            steer = 0.0

        speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
        steer = max(-STEER_MAG, min(STEER_MAG, steer))

    return ('', 204)

@app.route('/set_speed', methods=['POST'])
def set_speed():
    global MAX_SPEED
    with lock:
        MAX_SPEED = float(request.get_json()['value'])
    return ('', 204)

@app.route('/set_steer', methods=['POST'])
def set_steer():
    global STEER_MAG
    with lock:
        STEER_MAG = abs(float(request.get_json()['value']))
    return ('', 204)

# =====================
# ROS 2 node
# =====================
class DrivePublisher(Node):
    def __init__(self):
        super().__init__('phone_teleop_enhanced')
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            qos_profile_system_default
        )

    def run(self):
        rate = self.create_rate(PUBLISH_HZ)
        while rclpy.ok():
            with lock:
                v = speed
                s = steer

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = v
            msg.drive.steering_angle = s
            self.pub.publish(msg)

            rate.sleep()

# =====================
# Main
# =====================
def main():
    rclpy.init()
    node = DrivePublisher()

    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False),
        daemon=True
    ).start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
