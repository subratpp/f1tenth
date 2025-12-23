import rclpy, argparse
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# 1. Grab command line args
parser = argparse.ArgumentParser()
parser.add_argument('-s', '--speed', type=float, default=0.8)
parser.add_argument('-a', '--angle', type=float, default=0.0)
args, _ = parser.parse_known_args()

# 2. Initialize ROS 2 and Create Node
rclpy.init()
node = rclpy.create_node('tuner')

# 3. Create Pub/Sub
pub = node.create_publisher(AckermannDriveStamped, '/drive', 10)
sub = node.create_subscription(Odometry, '/odom', lambda m: print(f"X: {m.pose.pose.position.x:.2f} Y: {m.pose.pose.position.y:.2f} Z: {m.pose.pose.orientation.z:.2f}"), 10)

# 4. Drive Message
msg = AckermannDriveStamped()
msg.drive.speed, msg.drive.steering_angle = args.speed, args.angle

print(f"Running: Speed {args.speed}, Angle {args.angle}. Ctrl+C to stop.")

try:
    while rclpy.ok():
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
except KeyboardInterrupt:
    pub.publish(AckermannDriveStamped()) # Stop car
    node.destroy_node()
    rclpy.shutdown()