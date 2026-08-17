"""
Process A -- runs on the Tello side of the bridge.

Env required in the terminal that launches this:
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=10

Subscribes to /tello/marker_pose (published by aruco_detector.py) and
forwards each detection as JSON over a plain UDP socket to Process B
(process_b_astro_side.py), which lives in ASTRO's zenoh domain and cannot
otherwise see this topic. This process does no geometry -- it is a pure
passthrough so it stays simple and fast.
"""

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# If Process B runs on the SAME machine (recommended -- the dual-Wi-Fi
# desktop that can already reach both robots), 127.0.0.1 is correct.
# If it runs on a different host, replace with that host's IP.
RELAY_HOST = '127.0.0.1'
RELAY_PORT = 5599


class TelloPoseRelay(Node):
    def __init__(self):
        super().__init__('tello_pose_relay')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (RELAY_HOST, RELAY_PORT)

        self.sub = self.create_subscription(
            PoseStamped, '/tello/marker_pose', self.pose_cb, 10)

        self.get_logger().info(
            f"Relaying /tello/marker_pose -> udp://{RELAY_HOST}:{RELAY_PORT}")

    def pose_cb(self, msg: PoseStamped):
        payload = {
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'frame_id': msg.header.frame_id,
            'pos': [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            'quat': [msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w],
        }
        try:
            self.sock.sendto(json.dumps(payload).encode('utf-8'), self.target)
        except OSError as e:
            self.get_logger().warn(f"Failed to send relay packet: {e}")


def main():
    rclpy.init()
    node = TelloPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
