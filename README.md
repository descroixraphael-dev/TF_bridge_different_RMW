# Bridging Two ROS 2 Graphs on Different RMW Implementations

This document explains a general pattern for getting pose/tf information
across two ROS 2 graphs that **cannot see each other at the middleware
level** — e.g. one robot on `rmw_cyclonedds_cpp`, another on
`rmw_zenoh_cpp` (or `rmw_fastrtps_cpp`, or any other mismatched pair) —
using `process_a_tello_side.py` and `process_b_astro_side.py` as a
concrete, working example. If you're adapting this for a different pair
of robots, the pattern carries over directly; only the topic names,
frame names, and geometry in step 4 are specific to Tello/ASTRO.

---

## 1. The general problem

Two ROS 2 nodes can only discover each other automatically if they share
**both**:
- the same `ROS_DOMAIN_ID`, **and**
- a compatible RMW implementation (nodes on `rmw_cyclonedds_cpp` and
  `rmw_fastrtps_cpp` can sometimes interoperate since both speak DDS/RTPS;
  `rmw_zenoh_cpp` uses a different wire protocol entirely and is not
  bridgeable by standard DDS tools like `domain_bridge` or
  `zenoh-bridge-ros2dds`).

If either of those differs — different domain IDs, or fundamentally
different RMW implementations — normal ROS 2 discovery and pub/sub simply
won't cross that boundary, no matter what topic names you use.

A second, easy-to-miss constraint: `RMW_IMPLEMENTATION` is chosen **once,
per process**, at startup. A single Python process cannot hold a
cyclonedds context and a zenoh context simultaneously. So even if a
bridging tool existed for your specific RMW pair, you'd still need
separate processes, each native to one graph.

**The pattern:** run one small process inside each graph, and let them
exchange only the minimum data needed over something that isn't ROS at
all — a plain socket. Each process talks ROS normally to its own graph,
and talks JSON-over-UDP to its counterpart.

```
 GRAPH A (domain X, RMW #1)                  GRAPH B (domain Y, RMW #2)
 ───────────────────────────                 ───────────────────────────
 <source topic on graph A>                    <consumer of graph B's own tf>
        │                                              │
        ▼                                              ▼
 process_a  ────────── UDP / JSON ──────────►   process_b
 (native to graph A)                            (native to graph B)
                                                        │
                                                        ▼
                                                 broadcasts result into
                                                 graph B's own tf tree
```

This generalizes to any two robots, any two RMW implementations, and any
data you need to move across — pose, battery state, a command, whatever
fits in a small JSON payload and doesn't need ROS-level QoS guarantees.

---

## 2. `process_a_*` — the relay (lives in graph A)

**Role:** pure passthrough. No knowledge of graph B's frames, topics, or
even that graph B exists beyond an IP/port to send to.

**Generic shape:**
1. Subscribe to whatever source topic graph A needs to expose.
2. On each message, pack the relevant fields into JSON.
3. `sock.sendto()` that JSON to `(RELAY_HOST, RELAY_PORT)`.

**Why UDP:** this is meant for live, high-rate sensor/state streams where
a dropped packet is superseded by the next one moments later. UDP avoids
connection/retry overhead that would only add latency. If your data
genuinely can't tolerate any loss (a one-shot command, for instance), swap
in TCP instead — the rest of the pattern is unchanged.

**Concretely, in this repo:** `process_a_tello_side.py` subscribes to
`/tello/marker_pose` (`geometry_msgs/PoseStamped`, published by
`aruco_detector.py`, expressed in the Tello camera's optical-frame
convention) and relays it to `127.0.0.1:5599`. It runs under
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=10` — the Tello's
own environment.

---

## 3. `process_b_*` — the compute + publish node (lives in graph B)

**Role:** everything domain-specific happens here. It's native to graph
B, so it can freely use graph B's own tf tree, topics, and services — the
only "foreign" input is the JSON arriving over the socket.

**Generic shape:**
1. A background thread (not a ROS timer/callback) blocks on
   `sock.recvfrom()` and decodes each packet — a separate thread because
   a blocking socket call inside a ROS callback would stall
   `rclpy.spin()`.
2. Whatever geometry/logic is needed to turn "data from graph A" +
   "graph B's own current state" into the desired output.
3. Publish/broadcast that output natively into graph B (a tf broadcast, a
   topic publish, a service call — whatever graph B's consumers expect).

**Concretely, in this repo:** `process_b_astro_side.py` runs under
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `ROS_DOMAIN_ID=$ASTRO_NUM` — ASTRO's
own environment. For each relayed marker detection it:
1. Looks up ASTRO's own `map -> base_link` transform (native tf2 lookup,
   no bridging needed for this part — it's already in graph B).
2. Passes that, plus the raw marker pose, into
   `transforms.compose_drone_pose_in_map()`, which converts the marker
   pose from OpenCV's optical convention to ROS's body convention, then
   chains `map→astro_base_link → astro_base_link→marker →
   marker→camera → camera→tello_base_link` to get `map→tello_base_link`.
3. Broadcasts that as a `TransformStamped`, making `tello/base_link` a
   real frame in ASTRO's tf tree.
4. Separately (on its own timer, decoupled from step 1–3 so it doesn't
   blink out during detection gaps) publishes a `visualization_msgs/Marker`
   of type `MESH_RESOURCE` on `/tello/model_marker`, with
   `frame_locked=True` so it rides `tello/base_link` smoothly in rviz.

---

## 4. Tunable constants (ASTRO/Tello specifics)

All in `process_b_astro_side.py` and `transforms.py`:

| Constant | Symptom if wrong |
|---|---|
| `ASTRO_BASE_LINK_TO_MARKER` (`transforms.py`) | Whole drone position offset by a fixed amount |
| `TELLO_BASE_LINK_TO_CAMERA` (`transforms.py`) | Fixed rotation error |
| `MESH_SCALE` | Model way too big/tiny |
| `MESH_POSITION_OFFSET` | Model floats away from / sinks into the tf axis — scales linearly with `MESH_SCALE`, so re-scale both together |
| `MESH_ORIENTATION_OFFSET` | Model lying on its side, or nose pointing the wrong way |
| `RELAY_HOST`/`RELAY_PORT` (Process A) + `LISTEN_HOST`/`LISTEN_PORT` (Process B) | Nothing arrives at all — check these match |

---

## 5. Running it, start to finish (ASTRO/Tello)

1. **Graph A env** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `ROS_DOMAIN_ID=10`): start `driver_node.py` + `aruco_detector.py` so
   `/tello/marker_pose` is live.
2. **Graph B env** (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`,
   `ROS_DOMAIN_ID=$ASTRO_NUM`, correct `ZENOH_CONFIG_OVERRIDE`): bring up
   ASTRO's own stack so `map->base_link` is being published.
3. **Graph A env** + `source ~/bridge_ws/install/setup.bash`:
   ```bash
   ros2 run multi_robot_bridge tello_relay
   ```
4. **Graph B env** + `source ~/bridge_ws/install/setup.bash`:
   ```bash
   ros2 run multi_robot_bridge drone_tf_publisher
   ```
5. **Graph B env**: rviz2 with ASTRO's SLAM config — TF display
   auto-picks up `tello/base_link`; `Add → By topic →
   /tello/model_marker → Marker` shows the drone model.

Troubleshooting order: is Process A actually receiving its source topic?
Is Process B's socket actually receiving packets? Does graph B's own
reference tf (`map->base_link` here) exist yet? Is
`~/bridge_ws/install/setup.bash` sourced in *every* relevant terminal,
including the one running rviz?

---

## 6. Adapting this for a different robot pair

If you're reusing this pattern outside ASTRO/Tello, the parts that stay
identical are the two-process split, the UDP relay, and the
background-thread socket listener. What changes:

- **Process A**: subscribe to whatever topic on graph A you need to
  expose, and serialize whatever fields graph B needs — not necessarily
  a `PoseStamped`.
- **Process B**: replace the tf-lookup + `compose_drone_pose_in_map()`
  step with whatever geometry or logic is specific to your two robots'
  frame relationships. The optical→ROS correction in `transforms.py` is
  only relevant if your data comes from `cv2.solvePnP`-style camera
  detection — drop it if your source data is already in ROS convention.
  The static-offset constants (`*_TO_*` transforms) are still the right
  place to encode any fixed mounting/calibration offsets, whatever they
  are for your setup.
- **Networking**: `RELAY_HOST`/`RELAY_PORT` need to match your actual
  network layout — same idea as ASTRO/Tello (one relay talks to
  `127.0.0.1` if both processes share a machine, or a real IP if not).
