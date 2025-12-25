#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import curses
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ackermann_msgs.msg import AckermannDriveStamped


class TextWindow:
    def __init__(self, stdscr):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

    def read_key(self):
        key = self._screen.getch()
        return key if key != -1 else None

    def clear(self):
        self._screen.clear()

    def write(self, y, msg):
        h, w = self._screen.getmaxyx()
        self._screen.addstr(y, 2, msg.ljust(w))

    def refresh(self):
        self._screen.refresh()


class ToggleAckermannTeleop(Node):

    def __init__(self, ui):
        super().__init__('toggle_ackermann_teleop')
        self.ui = ui

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            qos_profile_system_default
        )

        # Fixed values
        self.SPEED_ON = 1.0
        self.STEER_MAG = 0.20
        self.HZ = 20.0

        # Current command state
        self.speed = 0.0
        self.steer = 0.0

        # Debounce to avoid key auto-repeat toggling multiple times
        self._last_key = None
        self._last_key_time = 0.0
        self._debounce_sec = 0.15

        self.running = True

    def _debounced(self, key):
        now = time.time()
        if key == self._last_key and (now - self._last_key_time) < self._debounce_sec:
            return True
        self._last_key = key
        self._last_key_time = now
        return False

    def run(self):
        while self.running:
            key = self.ui.read_key()
            if key is not None:
                if not self._debounced(key):
                    self._handle_key(key)

            self._publish()
            self._render()
            time.sleep(1.0 / self.HZ)

    def _handle_key(self, key):
        if key == ord('q'):
            self._stop()
            return

        # SPEED: additive with clipping
        if key == curses.KEY_UP:
            self.speed += 1.0
        elif key == curses.KEY_DOWN:
            self.speed -= 1.0

        # STEERING: additive with clipping
        elif key == curses.KEY_LEFT:
            self.steer += self.STEER_MAG
        elif key == curses.KEY_RIGHT:
            self.steer -= self.STEER_MAG

        # RESET
        elif key == ord(' '):
            self.speed = 0.0
            self.steer = 0.0

        # CLIP (this is the key part)
        self.speed = max(-1.0, min(1.0, self.speed))
        self.steer = max(-self.STEER_MAG, min(self.STEER_MAG, self.steer))


    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steer)
        self.pub.publish(msg)

    def _render(self):
        self.ui.clear()
        self.ui.write(1, f"Current speed    : {self.speed:.2f}")
        self.ui.write(2, f"Current steering : {self.steer:+.2f} rad")
        self.ui.write(4, "UP    : speed = 1.0")
        self.ui.write(5, "DOWN  : speed = 0.0")
        self.ui.write(6, "LEFT  : steering = +0.20 rad")
        self.ui.write(7, "RIGHT : steering = -0.20 rad")
        self.ui.write(8, "SPACE : speed = 0, steering = 0")
        self.ui.write(9, "q     : quit")
        self.ui.refresh()

    def _stop(self):
        self.running = False
        # publish stop once
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)
        os.kill(os.getpid(), signal.SIGINT)


def execute(stdscr):
    rclpy.init()
    node = ToggleAckermannTeleop(TextWindow(stdscr))
    node.run()
    node.destroy_node()
    rclpy.shutdown()


def main():
    curses.wrapper(execute)


if __name__ == '__main__':
    main()
