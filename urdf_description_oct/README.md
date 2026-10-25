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
8. [Quickstart & Verification Guide](#quickstart--verification-guide)

---

## Overview of Issues & Solutions

| Issue / Feature | Root Cause | Resolution | Impacted Files |
| :--- | :--- | :--- | :--- |
| **Teleop Movement Swapped / Inverted** | Right wheel joint axis was inverted (`[0, -1, 0]` vs `[0, 1, 0]`). | Aligned right wheel axis to `[0.0, 1.0, 0.0]`. | `urdf/October_2026_URDF.xacro` |
| **Spawn in World Environment** | Simulation previously spawned into default `empty.sdf`. | Added `turtlebot3_world.world`, set `GZ_SIM_RESOURCE_PATH`, set safe spawn position. | `worlds/turtlebot3_world.world`<br>`launch/gazebo.launch.py`<br>`setup.py` |
| **Gazebo Crash (`ItemIdentityException`)** | `gz-sim-sensors-system` was declared twice (in world AND in URDF). | Removed duplicate plugin from model URDF (handled at world level). | `urdf/October_2026_URDF.gazebo` |
| **Odometry Verification** | Clarification on `/odom` topic pipeline. | Verified `/odom` is produced by Gazebo DiffDrive plugin and bridged via `ros_gz_bridge`. | `urdf/October_2026_URDF.gazebo`<br>`config/ros_gz_bridge_gazebo.yaml` |
| **LiDAR Self-Reflection Clusters** | Lidar beams hit robot chassis/wheels at 4 repeatable sectors (0.22–0.28m). | Implemented `scan_filter` node; redirected Nav2 & RViz to `/scan_filtered`. | `urdf_description_oct/scan_filter.py`<br>`config/nav2_params.yaml`<br>`rviz/*.rviz` |
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
A diagnostic sweep identified 4 persistent clusters of returns at 0.22–0.28m caused by LiDAR self-intersection with robot chassis/caster components:
* **Sector A**: Beams 30–36 (`-149.92°` to `-143.90°`) @ 0.22–0.27 m
* **Sector B**: Beams 144–149 (`-35.60°` to `-30.58°`) @ 0.22–0.27 m
* **Sector C**: Beams 210–215 (`+30.58°` to `+35.60°`) @ 0.23–0.27 m
* **Sector D**: Beams 323–329 (`+143.90°` to `+149.92°`) @ 0.22–0.28 m

### Solution: `scan_filter` Node
Implemented a standalone, zero-dependency ROS 2 node (`scan_filter.py`):
1. **Configurable Parameters**:
   * `input_topic`: `/scan`
   * `output_topic`: `/scan_filtered`
   * `filter_sectors`: `[-150.0, -143.0, -36.0, -30.0, 30.0, 36.0, 143.0, 151.0]`
   * `sector_max_range`: `0.35` (Gated distance filtering: returns $< 0.35\text{ m}$ in these sectors are set to `inf`; returns $> 0.35\text{ m}$ remain visible to preserve obstacle detection).
   * `replacement_value`: `'inf'` (compatible with Nav2 costmap rayclearing).
2. **Crash-Safe Shutdown**:
   Guarded with `if rclpy.ok(): rclpy.shutdown()` and caught `KeyboardInterrupt` / `ExternalShutdownException` to eliminate `RCLError: failed to shutdown: rcl_shutdown already called`.
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
