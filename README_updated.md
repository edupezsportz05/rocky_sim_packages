# October 2026 AMR Simulation & Navigation Report

This repository contains the ROS 2 / Gazebo Sim packages for the differential drive AMR (Autonomous Mobile Robot), featuring **`urdf_description_oct`** (robot model, Gazebo simulation, sensor plugins, and LiDAR filtering) and **`my_robot_navigation`** (Nav2 autonomous navigation stack, AMCL localization, and costmaps).

---

## Table of Contents
1. [Overview of Issues & Solutions](#overview-of-issues--solutions)
2. [Issue 1: Teleoperation Motion Inversion](#issue-1-teleoperation-motion-inversion)
3. [Issue 2: World Environment & Spawning Integration](#issue-2-world-environment--spawning-integration)
4. [Issue 3: Gazebo OGRE Rendering Crash Resolution](#issue-3-gazebo-ogre-rendering-crash-resolution)
5. [Architecture: /odom Topic Dataflow](#architecture-odom-topic-dataflow)
6. [Issue 4: LiDAR Self-Reflection & Sector Filtering](#issue-4-lidar-self-reflection--sector-filtering)
7. [Package Configuration & Build Fixes](#package-configuration--build-fixes)
8. [Troubleshooting Session Report: LiDAR, Odometry & Nav2](#troubleshooting-session-report-lidar-odometry--nav2)
9. [Known Issues & Next Steps](#known-issues--next-steps)
10. [Quickstart & Verification Guide](#quickstart--verification-guide)

---

## Overview of Issues & Solutions

| Issue / Feature | Root Cause | Resolution | Impacted Files |
| :--- | :--- | :--- | :--- |
| **Teleop Movement Swapped / Inverted** | Right wheel joint axis was inverted (`[0, -1, 0]` vs `[0, 1, 0]`). | Aligned right wheel axis to `[0.0, 1.0, 0.0]`. | `urdf/October_2026_URDF.xacro` |
| **Spawn in World Environment** | Simulation previously spawned into default `empty.sdf`. | Added `turtlebot3_world.world`, set `GZ_SIM_RESOURCE_PATH`, set safe spawn position. | `worlds/turtlebot3_world.world`<br>`launch/gazebo.launch.py`<br>`setup.py` |
| **Gazebo Crash (`ItemIdentityException`)** | `gz-sim-sensors-system` was declared twice (in world AND in URDF). | Removed duplicate plugin from model URDF (handled at world level). | `urdf/October_2026_URDF.gazebo` |
| **Odometry Verification** | Clarification on `/odom` topic pipeline. | Verified `/odom` is produced by Gazebo DiffDrive plugin and bridged via `ros_gz_bridge`. | `urdf/October_2026_URDF.gazebo`<br>`config/ros_gz_bridge_gazebo.yaml` |
| **LiDAR Self-Reflection Clusters** | Lidar beams hit robot chassis/wheels at 4 repeatable sectors (0.22–0.28m). | Implemented `scan_filter` node; redirected Nav2 & RViz to `/scan_filtered`. | `urdf_description_oct/urdf_description_oct/scan_filter.py`<br>`my_robot_navigation/scripts/scan_filter.py`<br>`my_robot_navigation/config/nav2_params.yaml`<br>`my_robot_navigation/rviz/navigation.rviz` |
| **Script Install Path** | `setup.cfg` pointed scripts to `fusion2urdf_ros2`. | Corrected script destination to `urdf_description_oct`. | `setup.cfg` |

---

## Issue 1: Teleoperation Motion Inversion

### Symptom
When using `teleop_twist_keyboard`:
* `i` (Forward) caused the robot to **rotate right**.
* `,` (Backward) caused the robot to **rotate left**.
* `j` (Turn Left) caused the robot to **move backward**.
* `l` (Turn Right) caused the robot to **move forward**.

### Root Cause
In standard differential drive kinematics:
$$\dot{\theta}_{\text{left}} = \frac{v - \frac{\omega \cdot W}{2}}{R}, \quad \dot{\theta}_{\text{right}} = \frac{v + \frac{\omega \cdot W}{2}}{R}$$

In `urdf/October_2026_URDF.xacro`:
* Left Joint (`Revolute 1`): `<axis xyz="0.0 1.0 0.0"/>`
* Right Joint (`Revolute 2`): `<axis xyz="0.0 -1.0 0.0"/>` *(Inverted)*

Because the right wheel's axis pointed in the opposite direction (-Y), commanding positive rotational velocity ($v > 0$) caused the left wheel to spin forward and the right wheel to spin backward, creating clockwise in-place rotation (turning right). Commanding angular velocity ($\omega > 0$) commanded opposite signs to each wheel, which the inverted axis turned into identical backward rotation for both wheels.

### Fix
In `urdf/October_2026_URDF.xacro`, aligned the axis of `Revolute 2`:
```xml
<joint name="Revolute 2" type="continuous">
  <origin xyz="0.0 -0.198 0.065658" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="right_wheel_1"/>
  <axis xyz="0.0 1.0 0.0"/>
</joint>
```

---

## Issue 2: World Environment & Spawning Integration

### Changes Made
1. **Added World File**: Copied `turtlebot3_world.world` into `urdf_description_oct/worlds/`.
2. **Updated `setup.py`**: Added data file installation for `worlds/*`:
   ```python
   (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
   ```
3. **Updated `launch/gazebo.launch.py`**:
   * Configured `GZ_SIM_RESOURCE_PATH` to include TurtleBot3 Gazebo models:
     ```python
     tb3_models_path = os.path.join(pkg_tb3_gazebo, 'models')
     ```
   * Updated Gazebo startup arguments to load `turtlebot3_world.world`.
   * Adjusted robot spawn pose from `(0.0, 0.0, 0.32)` to `(-2.0, -0.5, 0.32)` to place the robot in open space away from the center cylinder obstacle.

---

## Issue 3: Gazebo OGRE Rendering Crash Resolution

### Symptom
When launching Gazebo with `turtlebot3_world.world`:
```text
[Err] [BaseStorage.hh:975] Another item already exists with name: scene
terminate called after throwing an instance of 'Ogre::ItemIdentityException'
  what(): OGRE EXCEPTION(4:ItemIdentityException): A material datablock with name '[Hash ...]' already exists.
```

### Root Cause
In Gazebo Sim (gz-sim), `gz-sim-sensors-system` manages the rendering thread and OGRE scene (`scene`). It is a **world-level system plugin**.
* `turtlebot3_world.world` already loaded `gz-sim-sensors-system`.
* `October_2026_URDF.gazebo` also contained `<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">`.

When the robot model was spawned, Gazebo tried to initialize a second rendering scene with the duplicate name `scene`, causing OGRE's High-Level Material System (Hlms) to crash with `ItemIdentityException`.

### Fix
Removed the redundant `<plugin filename="gz-sim-sensors-system">` block from `urdf/October_2026_URDF.gazebo`. Sensor rendering is handled by the world plugin.

---

## Architecture: /odom Topic Dataflow

The `/odom` topic pipeline is entirely managed by **`urdf_description_oct`**, while **`my_robot_navigation`** acts as a subscriber:

```
[Gazebo Simulation]
   └── gz::sim::systems::DiffDrive (in October_2026_URDF.gazebo)
         │ (publishes gz.msgs.Odometry)
         ▼
[Gazebo topic: /odom]
         │
         ▼
[ros_gz_bridge: parameter_bridge]
         │ (translates via config/ros_gz_bridge_gazebo.yaml)
         ▼
[ROS 2 topic: /odom] (nav_msgs/msg/Odometry)
         │
         ├──► Nav2 AMCL (Localization)
         ├──► Nav2 Controller Server / DWB Planner (Odometry tracking)
         ├──► Nav2 Local Costmap (Rolling window frame)
         └──► RViz Display
```

---

## Issue 4: LiDAR Self-Reflection & Sector Filtering

### Diagnostics
A diagnostic sweep identified 4 persistent clusters of returns at 0.22–0.28 m that are consistent with possible LiDAR self-returns from robot geometry. This is a strong candidate explanation, but the sectors should be checked against the robot CAD/visual geometry before treating the cause as definitively confirmed:
* **Sector A**: Beams 30–36 (`-149.92°` to `-143.90°`) @ 0.22–0.27 m
* **Sector B**: Beams 144–149 (`-35.60°` to `-30.58°`) @ 0.22–0.27 m
* **Sector C**: Beams 210–215 (`+30.58°` to `+35.60°`) @ 0.23–0.27 m
* **Sector D**: Beams 323–329 (`+143.90°` to `+149.92°`) @ 0.22–0.28 m

### Solution: `scan_filter` Node
The repository contains a standalone ROS 2 `LaserScan` filtering node (`scan_filter.py`):
1. **Configurable Parameters**:
   * `input_topic`: `/scan`
   * `output_topic`: `/scan_filtered`
   * `filter_sectors`: `[-150.0, -143.0, -36.0, -30.0, 30.0, 36.0, 143.0, 151.0]`
   * `sector_max_range`: `0.35` (Gated distance filtering: returns $< 0.35\text{ m}$ in these sectors are set to `inf`; returns $> 0.35\text{ m}$ remain visible to preserve obstacle detection).
   * `replacement_value`: `'inf'` (compatible with Nav2 costmap rayclearing).
2. **Shutdown handling**: catches `KeyboardInterrupt` / `ExternalShutdownException` and guards shutdown with `if rclpy.ok()` to reduce duplicate-shutdown errors.
3. **QoS Profile**:
   Subscribes and publishes with `qos_profile_sensor_data` (Best Effort).

### Updated Configuration Files
* **`my_robot_navigation/config/nav2_params.yaml`**:
  * AMCL: `scan_topic: scan_filtered`
  * Local Costmap Obstacle Layer: `topic: /scan_filtered`
  * Global Costmap Obstacle Layer: `topic: /scan_filtered`
  * Collision Monitor: `topic: "scan_filtered"`
* **`my_robot_navigation/rviz/navigation.rviz`**:
  * LaserScan topic set to `/scan_filtered`.
* **`urdf_description_oct/config/gazebo.rviz`**:
  * LaserScan topic set to `/scan_filtered`.

---

## Package Configuration & Build Fixes

* **Fixed `setup.cfg`**:
  Replaced legacy `script_dir=$base/lib/fusion2urdf_ros2` with:
  ```ini
  [develop]
  script_dir=$base/lib/urdf_description_oct
  [install]
  install_scripts=$base/lib/urdf_description_oct
  ```
  This allows `ros2 run urdf_description_oct scan_filter` to be discovered.
* **CMakeLists.txt (`my_robot_navigation`)**:
  Added script installation for `scan_filter.py`:
  ```cmake
  install(PROGRAMS scripts/scan_filter.py DESTINATION lib/${PROJECT_NAME})
  ```

---

## Quickstart & Verification Guide

### 1. Build the Workspace
```bash
cd /home/edupez/ros_ws
colcon build --packages-select urdf_description_oct my_robot_navigation
source install/setup.bash
```

### 2. Launch Simulation & LiDAR Filter
```bash
ros2 launch urdf_description_oct gazebo.launch.py
```
* Gazebo will launch `turtlebot3_world.world`.
* Robot spawns at `(-2.0, -0.5, 0.32)`.
* `scan_filter` automatically starts publishing `/scan_filtered`.

### 3. Verify Topics & Teleoperation
In a second terminal:
```bash
source /opt/ros/jazzy/setup.bash
source /home/edupez/ros_ws/install/setup.bash

# Check scan topics
ros2 topic echo /scan --once
ros2 topic echo /scan_filtered --once

# Teleoperate
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
* `i`: moves forward
* `,`: moves backward
* `j`: turns left
* `l`: turns right

### 4. Launch Autonomous Navigation
In a third terminal:
```bash
source /opt/ros/jazzy/setup.bash
source /home/edupez/ros_ws/install/setup.bash

ros2 launch my_robot_navigation navigation.launch.py
```
* Nav2 and RViz will launch with AMCL and costmaps listening to `/scan_filtered`.

---

## Troubleshooting Session Report: LiDAR, Odometry & Nav2

This section records the additional investigation performed during the October 2026 troubleshooting session. It separates observed evidence from hypotheses and work that remains to be verified.

### 1. LiDAR stream and scan rate

**Observed**
- The ROS 2 scan topic is `/scan`, with `sensor_msgs/msg/LaserScan` messages using `header.frame_id: lidar_1`.
- Each scan contains 360 beams. The scan spans approximately -180° to +180°, with an angular increment of approximately 1.0025°.
- The URDF/Gazebo sensor configuration sets `<update_rate>10</update_rate>`.
- Runtime measurements with `ros2 topic hz /scan` were approximately 4.9–5.0 Hz, below the configured 10 Hz target.
- The diagnostic subscriber successfully received repeated scans when run with `qos_profile_sensor_data`; the absence of output in the first script attempt was resolved by adding explicit scan-count reporting.

**Interpretation / follow-up**
The runtime rate does not match the configured sensor update rate. The exact cause has not yet been established. Check simulation real-time factor, Gazebo sensor performance, bridge behavior, and scan timestamps before assuming the sensor configuration alone is at fault.

### 2. Repeatable short-range sectors

Repeated scans showed 26 beams below 0.35 m, grouped into four stable sectors:

| Sector | Beam indices (zero-based) | Approximate angle range | Observed range |
|---|---:|---:|---:|
| A | 30–36 | -149.92° to -143.90° | about 0.22–0.28 m |
| B | 144–149 | -35.60° to -30.58° | about 0.22–0.28 m |
| C | 210–215 | +30.58° to +35.60° | about 0.22–0.28 m |
| D | 323–329 | +143.90° to +149.92° | about 0.22–0.28 m |

These clusters were stable over dozens of diagnostic scans. Their stability is consistent with fixed geometry near the robot, but the range data alone does not prove that they are chassis reflections. Verify the sectors against the simulated robot geometry and inspect the scan in RViz before using the filter in a safety-critical setting.

The existing filter implementation replaces readings with `inf` only when the beam is inside a configured sector **and** the reading is at or below `sector_max_range` (default 0.35 m). This preserves longer-range returns in the same sectors. The raw `/scan` topic remains unchanged and the filtered output is `/scan_filtered`.

### 3. Odometry and motion verification

The robot was observed moving in Gazebo while receiving a forward velocity command of 0.1 m/s. Two `/odom` samples then showed:

| Measurement | Sample 1 | Sample 2 |
|---|---:|---:|
| Simulation timestamp | 5.123 s | 7.703 s |
| X position | 0.1598 m | 0.4178 m |
| Linear X velocity | 0.100 m/s | 0.100 m/s |

This confirms that `/odom` was updating and its forward velocity agreed with the command during this test. It does not by itself prove that every TF timestamp or localization transform is correct, but it makes a completely stalled odometry stream unlikely.

### 4. Nav2 activation, goal submission, and transform warnings

**Observed**
- The navigation lifecycle manager reported that managed nodes were active, including `waypoint_follower` and `docking_server`.
- AMCL accepted an initial pose, but logged a transform lookup warning: requested time 18.368 s while the latest transform data was at 18.356 s (a 12 ms difference).
- RViz reported dropping a `lidar_1` scan with timestamp 10.500 s because its timestamp was earlier than all data in the TF cache.
- A terminal publication to `/goal_pose` used a `PoseStamped` with `header.stamp` equal to zero and did not produce visible evidence of a path.

**Interpretation / follow-up**
- Lifecycle activation is necessary but does not prove that a plan was generated or that the controller is moving the robot.
- The 12 ms AMCL extrapolation warning may be a small timing race; it is not sufficient evidence on its own to identify the root cause.
- The RViz scan timestamp warning is more concerning if the simulation clock was already around 18 s while the scan was stamped at 10.5 s. Check `/clock`, `/scan` timestamps, and whether Gazebo or ROS nodes were restarted independently before diagnosing a persistent clock mismatch.
- For a definitive navigation test, submit a goal through the `/navigate_to_pose` action, inspect the action result/feedback, and verify whether the planner publishes a path. A zero-stamped `/goal_pose` message is not the best diagnostic of the action server.

### 5. Diagnostic commands used

```bash
# Check scan rate and message timestamps
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic echo /clock --once

# Check odometry while the robot moves
ros2 topic echo /odom --once

# Inspect navigation action availability
ros2 action list -t | grep navigate_to_pose

# Check planner output (topic name can depend on Nav2 configuration/remappings)
ros2 topic echo /plan --once
```

A small standalone Python diagnostic subscriber was used to count short returns and print their beam indices, angles, and ranges. It was run directly with `python3` and did not require a package build. The script used for this investigation was a local diagnostic helper; it is not included as a tracked repository file in this archive.

---

## Known Issues & Next Steps

These items remain open based on the evidence available in this archive and the troubleshooting session:

1. **Explain the LiDAR rate discrepancy.** The sensor is configured for 10 Hz but was observed near 5 Hz. Check simulation real-time factor and the scan/clock timestamps.
2. **Validate self-return sectors visually.** Confirm that the four angular clusters correspond to robot geometry. Do not broaden the sectors without checking whether real obstacles could be hidden.
3. **Verify filtered-topic wiring at runtime.** Confirm that `/scan_filtered` is being published and that AMCL, both costmaps, Collision Monitor, and RViz subscribe to the intended topic. The Gazebo launch already starts the `urdf_description_oct` filter node; avoid starting a second filter instance unnecessarily.
4. **Resolve timestamp/TF consistency.** Compare `/clock`, `/scan.header.stamp`, `/odom.header.stamp`, and TF availability in one uninterrupted simulation run. A scan that is several seconds behind the transform cache can be dropped by RViz and can affect localization/costmap processing.
5. **Verify Nav2 goal planning end-to-end.** Confirm `/navigate_to_pose` is available, send an action goal, inspect action feedback/result, and inspect the planner output and controller logs.
6. **Review `my_robot_navigation/launch/navigation.launch.py`.** In the archived version, `map_yaml_file` references `LaunchConfiguration('my_map')`, while the declared launch argument is named `map`; additionally, the `bringup_launch.py` include does not forward the configured `map`, `params_file`, `use_sim_time`, or `autostart` launch arguments. Correct and test this before relying on the launch file to load the repository's map and Nav2 parameter file.

### Status summary

- **Verified during the session:** the robot moves in Gazebo; `/odom` changes and reports the commanded forward velocity; `/scan` publishes at roughly 5 Hz; four stable short-return sectors appear in repeated scans.
- **Present in the repository:** a sector-based `LaserScan` filter publishing `/scan_filtered`, Nav2 configuration references to the filtered topic, a robot footprint polygon, Gazebo DiffDrive odometry configuration, and a TurtleBot3 world launch setup.
- **Not yet verified as resolved:** the scan-rate discrepancy, stale scan/TF timestamp warnings, and successful Nav2 path generation to a goal.

