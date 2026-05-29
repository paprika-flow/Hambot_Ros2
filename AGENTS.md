# Hambot Project: Full Codebase Reference

## 1. Physical Robot Platform
- **SBC:** Raspberry Pi 4B (2GB RAM), headless Debian 12 (Bookworm)
- **Sensors:** 2D USB LiDAR (Slamtec) + OAK-D Lite depth camera
- **ROS 2:** Humble Hawksbill (containerized on Ubuntu 22.04)
- **Middleware:** CycloneDDS (`rmw_cyclonedds_cpp`), `ROS_DOMAIN_ID=30`
- **Real-robot workspace:** `cj_ros_ws/` (Docker on Pi, `docker exec -it hambot bash`)

## 2. Simulation Host
- **Machine:** MacBook Air M2, 16GB RAM — macOS
- **Sim workspace on Mac:** `~/hambot_sim_ws/` (bind-mounted into container)
- **Container:** ARM64 Ubuntu + VNC desktop at `http://localhost:6080/vnc.html`
- **Rendering:** `LIBGL_ALWAYS_SOFTWARE=1` — Mesa CPU render (bypass Apple Silicon OpenGL bugs)

---

## 3. Directory Tree

```
hambot_sim_ws/                          # Simulation workspace (on Mac host)
├── .env                                # Image version tag
├── docker-compose.yml                  # Service definition, ports, mounts, env
├── Dockerfile                          # Container image (tiryoh/ros2-desktop-vnc + Gazebo + deps)
├── build_docker.sh                     # Incremental image build script
├── workflow.md                         # Session workflow (start/compile/run/stop)
├── src/
│   ├── hambot_description/             # Robot URDF package
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   └── urdf/
│   │       ├── hambot.urdf.xacro       # Production robot model
│   │       └── base.urdf.xacro         # OLD placeholder (unused, dead code)
│   └── hambot_bringup/                 # Launch + perception + control package
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── launch/
│       │   └── sim_bringup.launch.py   # Main launch file
│       ├── worlds/
│       │   ├── campus_sidewalk.sdf     # Gazebo world (cross sidewalk + grass)
│       │   └── map_generator.py        # (unused helper)
│       └── hambot_bringup/
│           ├── sidewalk_segmenter.py   # Node: binary mask from seg cam
│           ├── voronoi_path_planner.py # Node: skeleton path from mask
│           └── centroid_navigator.py   # Node: P-controller centering + turns

cj_ros_ws/                              # Real-robot workspace (on Pi 4B)
├── Dockerfile                          # Minimal ROS 2 Humble image
├── docker-compose.yml
├── build_docker.sh
├── workflow.md                         # Pi workflow (exec into container)
└── docs/
    └── Docker_setup.md

AGENTS.md                               # This file (project reference)
.gitignore
```

---

## 4. File-by-File Breakdown

### 4.1 `hambot_sim_ws/Dockerfile`
- **Base:** `tiryoh/ros2-desktop-vnc:humble` (ARM64, includes Ubuntu + ROS 2 Humble + VNC desktop)
- **Installed apt packages:** `nano`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-slam-toolbox`, `ros-humble-ros-gz`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-xacro`, `ros-humble-cyclonedds-cpp`
- **Pip packages:** `scipy`, `shapely`, `opencv-python` (for Voronoi path planner)
- **Workspace dir:** `/home/ubuntu/hambot_sim_ws`
- **Shell config:** Auto-sources workspace `install/setup.bash` on login; defines `reset_sim` alias to teleport robot to spawn point via Ignition service

### 4.2 `hambot_sim_ws/.env`
- `ROBOT_IMAGE_TAG=1.1` — image version used in `docker-compose.yml`

### 4.3 `hambot_sim_ws/docker-compose.yml`
- Builds from `.`, tags as `hambot_sim:${ROBOT_IMAGE_TAG}`
- **Ports:** `6080:80` (VNC web access)
- **Volume:** `.` → `/home/ubuntu/hambot_sim_ws` (bind mount — code edits on Mac visible inside container)
- **Env:** `ROS_DOMAIN_ID=30`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `LIBGL_ALWAYS_SOFTWARE=1`
- **Security:** `seccomp=unconfined` (needed for OpenGL rendering in Docker)
- **SHM:** 512 MB (prevents browser UI crashes)
- **Command:** `sleep infinity` (container stays alive for interactive use)

### 4.4 `hambot_sim_ws/build_docker.sh`
- Incremental version bump script. Prompts for new version tag, runs `docker compose build` with new tag.

### 4.5 `hambot_sim_ws/workflow.md`
- Step-by-step dev loop: start container → open VNC → `colcon build` → `ros2 launch hambot_bringup sim_bringup.launch.py` → `rviz2` → `teleop_twist_keyboard`
- Try-on-the-fly install pattern: install packages interactively to test, then add to Dockerfile permanently at end of session

---

### 4.6 `hambot_description/package.xml`
- Standard ROS 2 package manifest. Build type `ament_cmake`. Placeholder description/license.

### 4.7 `hambot_description/CMakeLists.txt`
- Installs `urdf/` directory into `share/${PROJECT_NAME}`. No compiled code.

### 4.8 `hambot_description/urdf/hambot.urdf.xacro` — **Robot Model**
- **Materials:** blue (chassis), black (wheels/camera), grey (LiDAR/stand)
- **Inertia macros:** `box_inertia(m, w, h, d)`, `cylinder_inertia(m, r, h)` — solid-body approximations
- **Links & Joints:**
  - `base_footprint` → `base_link` (fixed, Z-offset = 71 mm ground clearance)
  - `base_link` — box 193×157×100 mm, 2.5 kg
  - 4 wheels (`front_left`, `front_right`, `rear_left`, `rear_right`) — continuous joints, `wheel_joint_x=0.051`, `wheel_joint_y=0.0995`, cylinder radius 45 mm, thickness 10 mm, 0.15 kg each. Axis Z (rotation for diff-drive)
  - `laser_frame` — cylinder visual shifted up 20 mm to sit on chassis top. Gazebo GPU LiDAR sensor: 640 samples, 360°, 12 m range, 10 Hz, topic `scan`
  - `camera_stand` — box 10×92×250 mm mast at rear (X=-0.0915). 0.15 kg
  - `camera_frame` — box 15×90×25 mm, mounted at Z=220 mm on mast, X-offset 12.5 mm forward. Two Gazebo sensors:
    - `rgbd_camera` — 640×480, 5 Hz, topic `camera`
    - `segmentation` — semantic labels, topic `/segmentation`, 5 Hz
- **Diff-Drive Plugin:** 4-wheel, `wheel_separation=0.199`, `wheel_radius=0.045`, `cmd_vel` topic, 30 Hz odom, frame `odom` → `base_footprint`
- **Joint State Publisher Plugin:** topic `joint_states`
- **All values derived from xacro properties** (not hardcoded)

### 4.9 `hambot_description/urdf/base.urdf.xacro` — **Dead Code**
- Old placeholder model with different dimensions (500×300×150 mm chassis, wheel radius 100 mm, etc.)
- Not referenced by any launch file. Kept for reference only.

---

### 4.10 `hambot_bringup/package.xml`
- Standard manifest. `ament_cmake`. Placeholder description.

### 4.11 `hambot_bringup/CMakeLists.txt`
- Installs `launch/` and `worlds/` directories
- Installs 3 Python scripts as executable nodes to `lib/${PROJECT_NAME}/`
- No compiled C++ code

### 4.12 `hambot_bringup/launch/sim_bringup.launch.py` — **Launch File**
Starts everything in order:
1. **Robot State Publisher** — processes `hambot.urdf.xacro`, publishes `/robot_description`, uses sim time
2. **Gazebo Ignition** — launches `campus_sidewalk.sdf` world with `-r` (auto-start)
3. **Spawn Robot** — creates hambot at X=-4.5, Y=0, Z=0.1 (south end of sidewalk)
4. **ROS-Gazebo Bridge** — 10 bidirectional topic bridges:
   - `/cmd_vel` (Twist)
   - `/odom` (Odometry)
   - `/scan` (LaserScan)
   - `/joint_states` (JointState)
   - `/tf` (TFMessage)
   - `/clock` (Clock)
   - `/camera/image` (Image)
   - `/camera/points` (PointCloud2)
   - `/segmentation/labels_map` (Image) — ROS 2 ← Gazebo
   - `/segmentation/colored_map` (Image) — ROS 2 ← Gazebo
   - `/camera/sidewalk_mask` (Image) — ROS 2 → Gazebo (debug feedback)
   - `/voronoi/debug_image` (Image) — ROS 2 → Gazebo (debug feedback)
5. **Sidewalk Segmenter Node** — `sidewalk_segmenter.py`
6. **Voronoi Path Planner Node** — `voronoi_path_planner.py`, params: `input_topic=/camera/sidewalk_mask`, `target_gray=255`, `resize=960×720`
7. **Centroid Navigator Node** — `centroid_navigator.py`

### 4.13 `hambot_bringup/worlds/campus_sidewalk.sdf` — **Simulation World**
- **Physics:** 1 ms step, real-time factor 1.0
- **Plugins:** physics, user commands, scene broadcaster, sensors (ogre2)
- **Sun:** directional light with shadows
- **Grass Plane (Label 2):** 15×15 m box, top at Z=0.0, green material, `Label` plugin with `label=2`
- **Sidewalk Network (Label 1):** Grey, top at Z=0.02 (2 cm curb lip). 5 model links, all with `Label` plugin `label=1`:
  - **Intersection:** 1.2×1.2 m center square at origin
  - **South Branch:** 5.0×1.2 m, centered at X=-3.1 (spans X=-5.6 to X=-0.6)
  - **North Branch:** 3.0×1.2 m, centered at X=2.1 (spans X=0.6 to X=3.6)
  - **East Branch:** 1.2×3.0 m, centered at Y=2.1
  - **West Branch:** 1.2×3.0 m, centered at Y=-2.1

### 4.14 `hambot_bringup/worlds/map_generator.py`
- Standalone script. Not used by launch. Likely generates SDF programmatically.

---

## 5. Perception & Control Pipeline (ROS 2 Nodes)

```
Gazebo Segmentation Camera
  │  topic: /segmentation/labels_map (RGB, R channel = label ID)
  ▼
sidewalk_segmenter.py
  │  extracts label == 1 → binary mono8 mask
  │  pub: /camera/sidewalk_mask
  ▼
voronoi_path_planner.py
  │  contour → Voronoi → skeleton → path selection
  │  pub: /voronoi/best_angle (Float32)
  │       /voronoi/area_difference (Float32)
  │       /voronoi/best_path (PoseArray, 2 poses)
  │       /voronoi/debug_image (rgb8, sent back to Gazebo)
  ▼
centroid_navigator.py
  │  subscribes /camera/sidewalk_mask (for centering)
  │  subscribes /navigator/command (for turns)
  │  state machine: FOLLOW_SIDEWALK / TURNING_LEFT
  │  pub: /cmd_vel (Twist)
  ▼
DiffDrive Plugin (in Gazebo)
```

### 5.1 `sidewalk_segmenter.py`
- **Input:** `/segmentation/labels_map` — 3-channel RGB, red channel = 8-bit semantic label
- **Processing:** numpy reshape → extract red channel → `label_map == 1` → binary (255/0)
- **Output:** `/camera/sidewalk_mask` — `mono8` encoding
- No cv_bridge — raw byte decode to avoid Python/C++ ABI conflicts

### 5.2 `voronoi_path_planner.py`
- **Input:** `/camera/sidewalk_mask` (mono8 or rgb8)
- **Pipeline (core logic):**
  1. Decode image natively (no cv_bridge)
  2. Resize to 960×720
  3. Threshold (binary if target_gray=255, else exact label match)
  4. Find external contour → Cartesian Y-flip → downsample to ~50 pts
  5. Build `scipy.spatial.Voronoi` from boundary points
  6. Clip to `shapely.geometry.Polygon` buffer
  7. `get_skeleton_lines()` — extract finite ridge vectors, merge collinear segments into skeleton lines, identify junction vertices
  8. `interpreting_skeletons()` — classify left/right side edges, compute triangle area difference (measures asymmetry), find candidate paths, select straightest forward path with temporal consistency
- **Output topics:**
  - `/voronoi/debug_image` — visual: grey fill + cyan skeleton + red side edges + green candidate paths + yellow best path
  - `/voronoi/best_angle` — angle of selected path (degrees)
  - `/voronoi/area_difference` — left-right area imbalance (percent)
  - `/voronoi/best_path` — PoseArray with 2 poses (start/end of selected path segment)
- **Temporal tracking:** `prev_best_path_coords` — remembers last selected path for smooth frame-to-frame continuity

### 5.3 `centroid_navigator.py`
- **Inputs:** `/camera/sidewalk_mask` (image), `/navigator/command` (String)
- **State machine:**
  - `FOLLOW_SIDEWALK`: ROI = rows 60-90% of image height → find mean X of white pixels → P-controller (`Kp=0.8`) steers to center. Linear speed 0.25 m/s
  - `TURNING_LEFT`: open-loop — 0.12 m/s forward + 0.6 rad/s CCW for 3.2 seconds, then back to FOLLOW
- **Output:** `/cmd_vel` (Twist)
- On shutdown: publishes zero Twist (stops motors)

---

## 6. Robot Mechanical Properties (URDF)

| Property | Value |
|---|---|
| Chassis size | 193 × 157 × 100 mm |
| Chassis mass | 2.5 kg |
| Wheel radius | 45 mm |
| Wheel thickness | 10 mm |
| Wheel mass | 0.15 kg |
| Wheel X offset | 51 mm (front/back) |
| Wheel Y offset | 99.5 mm (left/right) |
| Wheel separation | 199 mm (2 × Y offset) |
| Ground clearance | 71 mm (base_link Z offset) |
| LiDAR height | chassis top + 20 mm |
| Camera mast height | 250 mm (at X=-91.5 mm rear) |
| Camera height | 220 mm up mast, 12.5 mm forward |

---

## 7. Simulation World Geography

```
                     North (3m)
                        │
                   ┌────┼────┐
                   │    │    │
      West (3m) ───┤    ●    ├─── East (3m)
                   │inters.  │
                   └────┼────┘
                        │
                     South (5m)
                        │
                   Spawn: X=-4.5
```

- Grass (label 2): 15×15 m, Z=0.0
- Sidewalk (label 1): Z=0.02 (2 cm curb lip)
- Intersection: 1.2×1.2 m at origin
- Robot spawns at X=-4.5, Y=0, facing towards intersection
