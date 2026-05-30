# cj_hambot_ws — Development Workflow

## Workspace Layout

```
cj_hambot_ws/
├── src/                       # All ROS 2 packages (shared between sim and robot)
├── Dockerfile.sim             # Simulation image (VNC + Gazebo + desktop)
├── Dockerfile.robot           # Robot image (headless, real hardware drivers)
├── docker-compose.sim.yml     # Compose config for simulation
├── docker-compose.robot.yml   # Compose config for physical robot
└── workflow.md                # This file
```

---

## Two Workflows

| | Simulation | Physical Robot |
|---|---|---|
| **Machine** | Any PC (Mac, Linux) | Raspberry Pi 4B |
| **Compose file** | `docker-compose.sim.yml` | `docker-compose.robot.yml` |
| **Access** | Browser → `localhost:6080` | `docker exec -it hambot bash` |
| **Image tag** | Line 5 in `docker-compose.sim.yml` | Line 5 in `docker-compose.robot.yml` |

---

## Never Rebuilt — Only Rebuilt When

The Docker image is like installed software. You build it **once**, then start/stop the container as many times as you want. Rebuild the image **only** when you need to:

- Add a new system package (`apt install`)
- Add a pip package
- Change the base OS / ROS version
- Change compile-time dependencies

Editing Python files, launch files, URDF, configs, worlds — **none** of these need an image rebuild. They are bind-mounted into the container.

---

## Starting and Stopping Containers

### Simulation

```bash
cd cj_hambot_ws

# First time or after Dockerfile changes — build image
docker compose -f docker-compose.sim.yml build

# Start container (use this every session)
docker compose -f docker-compose.sim.yml up -d

# Mac Apple Silicon users — add the flag
LIBGL_ALWAYS_SOFTWARE=1 docker compose -f docker-compose.sim.yml up -d

# Stop container (end of session or before rebuilding)
docker compose -f docker-compose.sim.yml down
```

Open browser to `http://localhost:6080/vnc.html` for desktop.

### Robot (on Pi)

```bash
cd cj_hambot_ws

# First time or after Dockerfile changes — build image
docker compose -f docker-compose.robot.yml build

# Start container
docker compose -f docker-compose.robot.yml up -d

# Enter the running container
docker exec -it hambot bash

# Stop container
docker compose -f docker-compose.robot.yml down
```

---

## Session Workflow (Simulation)

### 1. Start the container

```bash
cd ~/cj_hambot_ws
docker compose -f docker-compose.sim.yml up -d
```

### 2. Open VNC desktop

Browser: `http://localhost:6080/vnc.html`

### 3. Write code on host, build inside container

Code edits happen on your host machine (VS Code or any editor). The `src/` folder is bind-mounted into the container — changes appear instantly inside.

Inside VNC terminal:

```bash
cd ~/cj_hambot_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Launch simulation

```bash
ros2 launch hambot_bringup sim_bringup.launch.py
```

In another VNC terminal, open RViz:

```bash
rviz2
```

### 5. Teleoperate

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 6. Stop at end of session

On host (outside container):

```bash
docker compose -f docker-compose.sim.yml down
```

---

## Session Workflow (Robot)

### 1. SSH into Pi, start container

```bash
ssh pi@hambot.local
cd ~/cj_hambot_ws
docker compose -f docker-compose.robot.yml up -d
```

### 2. Enter container

```bash
docker exec -it hambot bash
```

### 3. Build and run

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch hambot_bringup robot_bringup.launch.py
```

### 4. Stop

```bash
# Inside container: Ctrl+C the launch, then exit
exit
# On host:
docker compose -f docker-compose.robot.yml down
```

---

## Adding a New System Package — The Try-on-the-Fly Loop

This is the most important pattern. Never add packages to the Dockerfile blindly. Always test first.

### Step 1 — Install interactively inside the running container

```bash
# Inside container (VNC terminal or docker exec)
sudo apt update && sudo apt install -y ros-humble-slam-toolbox
```

### Step 2 — Test it works

Run the new node, verify it does what you expect. Keep the container running — the `apt install` is live immediately.

### Step 3 — If it works, save it to the Dockerfile

Open the appropriate Dockerfile on your host:

| What you installed | Which Dockerfile |
|---|---|
| Sim-only package (Gazebo, RViz, GUI) | `Dockerfile.sim` |
| Robot hardware driver (LiDAR, motors, IMU) | `Dockerfile.robot` |
| Python pip package used everywhere | Both |
| General ROS 2 tool used everywhere | Both |

**Add new packages near the bottom** of the `apt install` block, not at the top. Why:

- Adding at the top shifts all subsequent layers → invalidates Docker build cache → forces full rebuild
- Adding at the bottom reuses cached layers for everything above → rebuild only the changed line

### Step 4 — Do NOT rebuild yet

Keep working. The interactive install stays alive until you stop the container. Rebuild only when you're ready to permanently bake it in (typically end of session, or next time you need a fresh container).

---

## Baking Changes into the Image

Only do this when you've confirmed your Dockerfile changes work and you want them permanent.

### 1. Bump the version tag

Open the compose file and increment the version number:

```yaml
# docker-compose.sim.yml
image: hambot_sim:1.2    # was 1.1 — bump it
```

Same for `docker-compose.robot.yml` if you changed that Dockerfile.

**Why bump:** Docker tags are immutable. If you don't change the tag, `docker compose build` overwrites the old image silently. Bumping gives you a rollback point — `docker compose -f docker-compose.sim.yml up -d` with the old tag still works.

### 2. Rebuild

```bash
cd ~/cj_hambot_ws
docker compose -f docker-compose.sim.yml build
```

### 3. Start fresh container with new image

```bash
docker compose -f docker-compose.sim.yml down
docker compose -f docker-compose.sim.yml up -d
```

The new container runs your updated image. All packages from the Dockerfile are now baked in — no need to `apt install` them again.

---

/prev
```
┌─ Every session ─────────────────────────────────────┐
│ docker compose -f docker-compose.sim.yml up -d       │
│ (Mac: prepend LIBGL_ALWAYS_SOFTWARE=1)               │
│                                                      │
│ Inside VNC: colcon build --symlink-install            │
│                                                      │
│ docker compose -f docker-compose.sim.yml down         │
└──────────────────────────────────────────────────────┘

┌─ Rarely: need new system package ───────────────────┐
│ 1. sudo apt install ros-humble-xyz  (inside container)│
│ 2. Test it works                                      │
│ 3. Add to Dockerfile (at bottom of apt block)         │
│ 4. Keep working, do NOT rebuild yet                   │
│ 5. End of session: bump tag, rebuild, restart         │
└──────────────────────────────────────────────────────┘
```
