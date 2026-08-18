# TF_bridge_different_RMW
# `multi_robot_bridge` — Protocol & File Reference

This document explains what `process_a_tello_side.py` and
`process_b_astro_side.py` each do, how they talk to each other, and how
the whole pipeline turns an ArUco detection into a drone model moving
around ASTRO's map in rviz.

---

## 1. Why this exists

ASTRO and the Tello live on two ROS 2 graphs that **cannot see each
other**:

| | ASTRO | Tello |
|---|---|---|
| RMW implementation | `rmw_zenoh_cpp` | `rmw_cyclonedds_cpp` |
| `ROS_DOMAIN_ID` | `ASTRO_NUM` (e.g. `3`) | `10` |

Different domain IDs alone would be solvable with `domain_bridge`, but
`rmw_zenoh_cpp` and `rmw_cyclonedds_cpp` are different wire protocols —
`rmw_zenoh` doesn't interoperate with the DDS-based bridging tools
(`domain_bridge`, `zenoh-bridge-ros2dds`). A single Python process also
can't hold two RMW contexts at once (`RMW_IMPLEMENTATION` is chosen once,
per process, at startup).

So the package runs **two separate processes**, each native to its own
graph, that talk to each other over a plain UDP socket instead of ROS
topics. That socket hop is the entire "bridge."

---

## 2. Architecture at a glance

```
 TELLO SIDE (domain 10, cyclonedds)          ASTRO SIDE (domain ASTRO_NUM, zenoh)
 ─────────────────────────────────           ──────────────────────────────────
 aruco_detector.py                            ASTRO's own SLAM stack
   publishes /tello/marker_pose                 publishes map -> base_link (tf)
        │                                              │
        ▼                                              ▼
 process_a_tello_side.py                       process_b_astro_side.py
   subscribes /tello/marker_pose      UDP        listens on socket
   serializes to JSON            ───────────►    looks up map->base_link
   sends to 127.0.0.1:5599                       composes full chain
                                                  broadcasts map->tello/base_link
                                                  publishes /tello/model_marker (mesh)
                                                              │
                                                              ▼
                                                       rviz2 (ASTRO domain)
                                                       shows drone mesh moving on map
```

Everything left of the UDP arrow runs under `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
/ `ROS_DOMAIN_ID=10`. Everything right of it runs under
`RMW_IMPLEMENTATION=rmw_zenoh_cpp` / `ROS_DOMAIN_ID=$ASTRO_NUM`. The two
environments never mix in the same terminal or process.

---

## 3. `process_a_tello_side.py` — the relay (Tello side)

**Role:** pure passthrough. No geometry, no ROS-domain knowledge of
ASTRO. Its only job is to get `/tello/marker_pose` out of the Tello's
cyclonedds domain and onto the wire in a format the other side can read.

**What it does, in order:**
1. Subscribes to `/tello/marker_pose` (`geometry_msgs/PoseStamped`,
   published by `aruco_detector.py`) — this is the marker's pose in the
   Tello camera's **optical** frame convention (x-right, y-down,
   z-forward), straight out of `cv2.solvePnP`.
2. On every message, packs the timestamp, position, and quaternion into
   a small JSON dict.
3. Sends that JSON as UTF-8 bytes over a UDP socket to
   `RELAY_HOST:RELAY_PORT` (default `127.0.0.1:5599`).

**Why UDP and not TCP:** this is a live sensor stream, not a control
command — if a packet is dropped, the next detection (arriving ~30Hz)
supersedes it anyway. UDP avoids the connection/retry bookkeeping that
would add latency for no benefit here. If you ever run Process A and B on
two different machines instead of one desktop, this is also the only
line you need to change (`RELAY_HOST`).

**What it does *not* do:** no coordinate math, no tf lookups, no ASTRO
awareness at all. Keeping it this thin means it can run entirely inside
the Tello's own domain/environment with zero dependency on ASTRO being
up, reachable, or even powered on.

---

## 4. `process_b_astro_side.py` — the compute + publish node (ASTRO side)

This is where the actual work happens. It has three independent jobs
running concurrently:

### 4a. Socket listener (background thread)
A daemon thread loops on `sock.recvfrom()`, decodes each incoming JSON
packet, and calls `handle_marker_pose()`. It runs in its own thread
(not the ROS executor) because `recvfrom()` is a blocking call — putting
it on a timer callback would stall `rclpy.spin()` between packets.

### 4b. `handle_marker_pose()` — the geometry
For each relayed detection:
1. Looks up ASTRO's own `map -> base_link` transform from `tf_buffer`
   (this is **native** — ASTRO already publishes this on its own domain,
   no bridging needed for this part).
2. Passes that transform, plus the raw marker position/quaternion, into
   `transforms.compose_drone_pose_in_map()`, which:
   - Converts the marker pose from OpenCV's optical-frame convention to
     ROS's body convention (x-forward, y-left, z-up) — this is the fixed
     `OPTICAL_TO_ROS` rotation.
   - Inverts it to get "camera pose relative to marker" instead of
     "marker pose relative to camera."
   - Chains: `map→astro_base_link → astro_base_link→marker →
     marker→camera → camera→tello_base_link` to arrive at
     `map→tello_base_link`.
3. Packs the result into a `TransformStamped` and broadcasts it with
   `tf2_ros.TransformBroadcaster` — this is what makes `tello/base_link`
   a real frame in ASTRO's tf tree, visible to any rviz/node running on
   that domain.

This only fires **when a detection arrives** — if the marker goes out of
view, this stops updating and the frame holds its last broadcast pose
(tf2 doesn't extrapolate past its buffer window).

### 4c. `publish_mesh_marker()` — the visual (independent timer, 0.5s)
Deliberately decoupled from 4b so the drone model doesn't disappear
during brief detection gaps. Publishes a `visualization_msgs/Marker` of
type `MESH_RESOURCE` on `/tello/model_marker`, pointing at
`package://multi_robot_bridge/meshes/drone.stl`.

Key fields and why they're set the way they are:

| Field | Value | Why |
|---|---|---|
| `header.frame_id` | `"tello/base_link"` | Marker's pose is expressed *relative to* this frame, not `map` directly |
| `frame_locked` | `True` | Tells rviz to re-resolve the marker's pose from the live tf tree every render frame, so it rides the moving frame smoothly instead of only updating every 0.5s |
| `scale.{x,y,z}` | `0.001` | STL was authored in millimeters; rviz assumes meters |
| `pose.position` | `MESH_POSITION_OFFSET` | Compensates for the mesh's local origin not being at its centroid |
| `pose.orientation` | `MESH_ORIENTATION_OFFSET` | Rotates the mesh's authored axes into ROS's forward/up convention |
| `mesh_use_embedded_materials` | `False` | STL carries no color data, so this tells rviz to use `m.color` instead of looking for (and failing to find) material info |
| `lifetime.sec` | `0` | Persists until explicitly replaced — doesn't auto-expire between timer ticks |

Because 4b and 4c both key off `DRONE_CHILD_FRAME = 'tello/base_link'`,
changing that one constant moves both the tf broadcast and the mesh
together — they're intentionally never allowed to drift apart into two
different frame names.

---

## 5. Tunable constants — where to look when something looks wrong

All in `process_b_astro_side.py`, near the top:

| Constant | Symptom if wrong |
|---|---|
| `ASTRO_BASE_LINK_TO_MARKER` (in `transforms.py`) | Whole drone position offset by a fixed amount, regardless of where it actually is |
| `TELLO_BASE_LINK_TO_CAMERA` (in `transforms.py`) | Same, but the offset would only show up as a fixed *rotation* error |
| `MESH_SCALE` | Model way too big/tiny |
| `MESH_POSITION_OFFSET` | Model floats away from / sinks into the tf axis |
| `MESH_ORIENTATION_OFFSET` | Model lying on its side, or nose pointing the wrong way |
| `RELAY_HOST` / `RELAY_PORT` (Process A) + `LISTEN_HOST` / `LISTEN_PORT` (Process B) | No mesh/tf appears at all — check these match |

---

## 6. Running the whole protocol, start to finish

Order matters for the first four; 5 and 6 can start any time after 4.

1. **Tello env** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `ROS_DOMAIN_ID=10`): start `driver_node.py` and `aruco_detector.py` so
   `/tello/marker_pose` is live.
2. **ASTRO env** (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`,
   `ROS_DOMAIN_ID=$ASTRO_NUM`, correct `ZENOH_CONFIG_OVERRIDE`): bring up
   ASTRO's own stack (`rsp.launch.py` + SLAM), so `map->base_link` is
   being published.
3. **Same Tello env** + `source ~/bridge_ws/install/setup.bash`:
   ```bash
   ros2 run multi_robot_bridge tello_relay
   ```
4. **Same ASTRO env** + `source ~/bridge_ws/install/setup.bash`:
   ```bash
   ros2 run multi_robot_bridge drone_tf_publisher
   ```
5. **ASTRO env**: launch/keep open rviz2 with the SLAM config
   (`astro_navigation/rviz/astro_online_async_rviz_config.rviz`) — the TF
   display auto-picks up `tello/base_link`, and `Add → By topic →
   /tello/model_marker → Marker` shows the drone model.
6. Fly the drone, walk it past the marker, watch it move on the map.

If it's ever not showing up, check in this order: is `tello_relay`
actually receiving `/tello/marker_pose` (echo the topic on the Tello
side)? Is `drone_tf_publisher`'s socket actually receiving packets (add a
log line if unsure)? Does ASTRO's own `map->base_link` tf exist yet
(`ros2 run tf2_ros tf2_echo map base_link` on the ASTRO domain)? Is
`~/bridge_ws/install/setup.bash` sourced in *every* terminal that needs
it, including the one running rviz?
