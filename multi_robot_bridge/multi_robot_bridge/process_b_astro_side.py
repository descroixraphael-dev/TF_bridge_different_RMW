"""
Process B -- runs on the ASTRO side of the bridge.

Env required in the terminal that launches this (match whatever your ASTRO
setup already uses, e.g. ASTRO_NUM=3):
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ASTRO_NUM=3
    export ROS_DOMAIN_ID=$ASTRO_NUM
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/192.168.8.1${ASTRO_NUM}:7447\"]"

Listens on a UDP socket for marker-pose detections relayed by
process_a_tello_side.py, looks up ASTRO's own map->base_link transform,
composes the full chain in transforms.py, and broadcasts the result as
map -> tello/base_link on ASTRO's own tf tree.
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from multi_robot_bridge import transforms as tf_math

LISTEN_HOST = '0.0.0.0'
LISTEN_PORT = 5599

MAP_FRAME = 'map'
ASTRO_BASE_FRAME = 'base_link'
DRONE_CHILD_FRAME = 'tello/base_link'

TF_LOOKUP_TIMEOUT_SEC = 0.2

# --- Drone mesh marker -------------------------------------------------
# STL bounding box came out to roughly 90 x 37 x 100 units, which matches
# a Tello's real ~98x92.5x41mm footprint -- so the mesh is authored in
# millimeters and needs a 0.001 scale to render at the correct size in
# rviz (which assumes meters). The mesh's own origin sits at its lowest
# point (z_min = 0), not its centroid -- if it looks like it's floating
# or sunk relative to the tf axis, that's why; nudge MESH_POSITION_OFFSET
# below to compensate.
MESH_RESOURCE_URI = 'package://multi_robot_bridge/meshes/drone.stl'
MESH_SCALE = 0.002
# If the model faces the "wrong" direction once you see it move, adjust
# this quaternion [x, y, z, w] to rotate the mesh into the frame's +x
# forward, +z up convention -- identity assumes it's already authored
# that way.
MESH_ORIENTATION_OFFSET = [0.7071068, 0.0, 0.0, 0.7071068]
MESH_POSITION_OFFSET = [-0.0906, 0.0998, -0.0370]
MESH_MARKER_PERIOD_SEC = 0.5


class DroneTfPublisher(Node):
    def __init__(self):
        super().__init__('drone_tf_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # Mesh marker: published on its own timer, decoupled from detection
        # arrival, so the drone model stays visible in rviz even during
        # brief gaps in marker detection. frame_locked=True tells rviz to
        # re-resolve the marker's pose from the current TF tree every
        # render frame rather than baking in the pose at publish time --
        # this is what lets it ride the moving tello/base_link frame
        # smoothly instead of only updating each time this timer fires.
        self.mesh_pub = self.create_publisher(Marker, '/tello/model_marker', 10)
        self.create_timer(MESH_MARKER_PERIOD_SEC, self.publish_mesh_marker)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LISTEN_HOST, LISTEN_PORT))
        self.sock.settimeout(1.0)

        self._stop = threading.Event()
        self._listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listener_thread.start()

        self.get_logger().info(
            f"Listening for relayed marker poses on udp://{LISTEN_HOST}:{LISTEN_PORT}, "
            f"will broadcast {MAP_FRAME} -> {DRONE_CHILD_FRAME}")

    def _listen_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(data.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                self.get_logger().warn(f"Bad relay packet: {e}")
                continue
            self.handle_marker_pose(payload)

    def handle_marker_pose(self, payload):
        try:
            t_map_astro = self.tf_buffer.lookup_transform(
                MAP_FRAME, ASTRO_BASE_FRAME, Time(),
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_SEC))
        except Exception as e:
            self.get_logger().warn(
                f"No {MAP_FRAME}->{ASTRO_BASE_FRAME} tf available yet: {e}",
                throttle_duration_sec=2.0)
            return

        T_map_astro_base_link = tf_math.ros_transform_to_matrix(t_map_astro.transform)

        T_map_tello_base_link = tf_math.compose_drone_pose_in_map(
            T_map_astro_base_link,
            marker_pos=payload['pos'],
            marker_quat=payload['quat'],
        )

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = MAP_FRAME
        out.child_frame_id = DRONE_CHILD_FRAME
        tf_math.fill_ros_transform_from_matrix(out.transform, T_map_tello_base_link)

        self.broadcaster.sendTransform(out)

    def publish_mesh_marker(self):
        m = Marker()
        m.header.frame_id = DRONE_CHILD_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'tello_drone'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = MESH_RESOURCE_URI
        m.mesh_use_embedded_materials = False
        m.frame_locked = True

        m.pose.position.x, m.pose.position.y, m.pose.position.z = MESH_POSITION_OFFSET
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = MESH_ORIENTATION_OFFSET

        m.scale.x = m.scale.y = m.scale.z = MESH_SCALE

        # STL carries no color info, so set one explicitly (light gray).
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.0, 1.0

        m.lifetime.sec = 0  # 0 = persists until replaced/deleted

        self.mesh_pub.publish(m)

    def destroy_node(self):
        self._stop.set()
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DroneTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
