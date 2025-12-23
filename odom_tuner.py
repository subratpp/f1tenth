import rclpy, argparse, math
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# 1. Grab command line args
parser = argparse.ArgumentParser()
parser.add_argument('-s', '--speed', type=float, default=0.9)
parser.add_argument('-a', '--angle', type=float, default=0.36)
args, _ = parser.parse_known_args()

# 2. Initialize ROS 2
rclpy.init()
node = rclpy.create_node('tuner')

# 3. Yaw extraction
def odom_cb(m):
    q = m.pose.pose.orientation
    # quaternion -> yaw
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    print(f"X: {m.pose.pose.position.x:.2f} "
          f"Y: {m.pose.pose.position.y:.2f} "
          f"Yaw: {yaw:.3f} rad")

# 4. Pub/Sub
pub = node.create_publisher(AckermannDriveStamped, '/drive', 20)
sub = node.create_subscription(Odometry, '/odom', odom_cb, 10)

# 5. Drive command
msg = AckermannDriveStamped()
msg.drive.speed = args.speed
msg.drive.steering_angle = args.angle

print(f"Running: Speed {args.speed}, Angle {args.angle}. Ctrl+C to stop.")

try:
    while rclpy.ok():
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pub.publish(AckermannDriveStamped())  # stop
    node.destroy_node()
    rclpy.shutdown()
