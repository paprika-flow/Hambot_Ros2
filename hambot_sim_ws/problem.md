# Hambot: Autonomous Sidewalk Navigation on College Campus

## 1. Project Context

### The Robot
- **Platform:** Custom 4-wheel differential drive robot
- **SBC:** Raspberry Pi 4B (2 GB RAM, headless Debian 12)
- **Sensors:** 2D LiDAR (Slamtec, 12 m range, 360°) + OAK-D Lite depth camera
- **Chassis:** 193×157×100 mm, ground clearance 71 mm
- **Wheels:** 45 mm radius, 199 mm separation
- **Max speed:** ~0.25 m/s (walking pace)

### The Environment
- **University of South Florida (USF) campus** — outdoor sidewalks
- **Terrain types:**
  - **Corridors:** Standard 1.2-2 m wide sidewalks with clear edges (grass/curb)
  - **Plazas:** Large open concrete areas (e.g., outside Marshall Student Center, library plaza). Open, no clear boundaries, pedestrians, tables, benches
  - **Crosswalks:** Marked pedestrian crossings at roads (heavy traffic nearby)
  - **Intersections:** Sidewalk junctions — 3-way, 4-way, sometimes 5-way (asymmetric)
  - **Bottlenecks:** Narrow passes between buildings (tight clearance, <1 m)
- **Dynamic obstacles:** Pedestrians, cyclists, skateboards, scooters, open doors, construction barriers
- **Lighting:** Full sun to deep shadows between buildings. Glare, rain, night
- **Surface:** Concrete (smooth to rough/cracked), curb ramps, drainage grates

### Constraints
- **RPi 4B 2 GB** — limited compute. No GPU. Must keep inference light
- **OAK-D** can run ML models on-device (VPU), offloading RPi
- **Real-time:** Must react at ~10 Hz (controller), ~5 Hz (perception)
- **No GPS** (or unreliable under trees/buildings). Must use visual + LiDAR odometry
- **No pre-built map** — or at least not a detailed one. Could have rough building/walkway topology

---

## 2. The Problem

### Core Mission
Navigate autonomously from **Building A to Building B** on campus sidewalks, handling plazas, intersections, obstacles, and varying terrain — without explicit per-case programming.

### What Makes This Hard

#### A. Not a Simple Corridor
Unlike indoor hallways, campus has **open plazas** where "follow the sidewalk" is meaningless. The robot must cross a wide concrete area with no walls. The only hint is the drivable surface (concrete vs grass) and the destination direction.

Traditional corridor-following methods (centroid tracking, Voronoi skeleton) break here — there's no single path, just an open field with obstacles.

#### B. Ambiguous Intersections
A 4-way intersection is simple. But campuses have:
- 5-way intersections (who is "left" when there are two leftward branches?)
- T-junctions with curved paths
- Offset intersections (branch doesn't meet at center)
- Sidewalks that merge gradually

Simple semantic commands like "turn left" are insufficient. The robot needs to understand **which branch** visually and match it to the global route.

#### C. Dynamic, Unstructured Obstacles
Pedestrians don't follow rules. They:
- Walk in groups (blocking entire sidewalk)
- Cut diagonally across plazas
- Stand in groups talking (partially blocking)
- Approach from unexpected angles (bikes, scooters)

A reactive planner that only centers on the sidewalk will get stuck or behave erratically.

#### D. Surface Semantics vs. Navigation Goal
The segmentation camera tells us: "this pixel is sidewalk (label 1), this is grass (label 2)." But it doesn't tell us:
- Which direction to go at an intersection
- Which branch leads to our destination
- Whether a crosswalk is safe to enter
- Where the curb ramp is

We need a **layer of intention** above raw perception.

#### E. Global-to-Local Coordination
The global planner (Building A → B) produces something like:
1. Go south on sidewalk 50 m
2. Turn right at 4-way intersection
3. Cross plaza heading southwest 30 m
4. Turn left at crosswalk

The local planner must **interpret** these instructions in real-time based on what it actually sees. If the intersection is actually 5-way, "turn right" must map to the correct physical branch.

---

## 3. Proposed Solution: Semantic Guidance Costmap

### Philosophy
Instead of hardcoding rules for each scenario, build a **unified cost function** that any path planner (A*) can solve. The costmap layers encode different considerations. The planner naturally handles all cases because the cost function smoothly guides the robot.

No if-else per scenario. The same A* handles corridors, plazas, intersections, and obstacle avoidance.

### Architecture

```
┌─────────────────┐
│  Global Planner  │  "Go 50m south, turn right at next intersection,
│  (to be built)   │   cross plaza to building B"
└────────┬────────┘
         │ Route (list of waypoints + semantic context)
         ▼
┌─────────────────────────────┐
│  Semantic Guidance Layer    │
│  Translates global plan     │
│  into local "guidance cost" │
└────────┬────────────────────┘
         │ Local goal + direction bias
         ▼
┌─────────────────────────────────────┐
│         Multi-Layer Costmap         │
│  ┌─────────┐ ┌────────┐ ┌────────┐ │
│  │Drivable │ │Obstacle│ │Guidance│ │
│  │Surface  │ │Layer   │ │Field   │ │
│  │(camera) │ │(LiDAR) │ │(global │ │
│  │         │ │        │ │intent) │ │
│  └─────────┘ └────────┘ └────────┘ │
│         │         │         │       │
│         ▼         ▼         ▼       │
│     Weighted Sum → Final Costmap    │
└────────────────┬────────────────────┘
                 │
                 ▼
         ┌──────────────┐
         │  A* Planner   │
         │  (local, 3-5m)│
         └──────┬───────┘
                │ Path (poses)
                ▼
         ┌──────────────┐
         │  Pure Pursuit │  or simple angle follower
         │  Controller   │
         └──────┬───────┘
                │ cmd_vel
                ▼
            Robot
```

### Costmap Layers — In Detail

#### Layer 1: Drivable Surface Cost
**Source:** Semantic segmentation camera (OAK-D / simulation)

| Cell content | Cost | Notes |
|---|---|---|
| Sidewalk (label 1) | 0 | Perfectly drivable |
| Plaza/road (custom label) | 1-5 | Drivable but caution |
| Grass (label 2) | 200-254 | Undesirable but traversable if needed |
| Unknown (outside FOV) | 254 | Assume unsafe |
| Building/wall obstacle | 255 | Lethal — must avoid |

**Edge awareness:** Cells near the boundary between drivable and non-drivable get a gradient cost (0 at center, ~50 at edge). This naturally keeps the robot away from edges without explicit "center-of-path" logic.

#### Layer 2: Obstacle Cost
**Source:** LiDAR (2D scan) + optionally depth camera

- Inflate each laser hit by robot radius (~15 cm) with linear decay
- Dynamic obstacles (moving) get temporary high cost
- Static obstacles (walls, poles) persist for seconds

**Fusion with Layer 1:** If LiDAR shows an obstacle on the sidewalk, that cell becomes lethal regardless of Layer 1. This allows the robot to briefly leave the sidewalk (drive on grass edge) to bypass a pedestrian blocking the path.

#### Layer 3: Global Guidance Field
**Source:** Semantic Guidance Layer (translates global plan)

This is the innovation that handles intersection ambiguity and plazas.

Instead of a single goal point, the guidance field provides a **potential field** toward the intended direction:

- A **linear corridor bias**: cells in the intended travel direction get reduced cost
- At intersections, the correct branch gets a "cost valley" pointing to it
- In plazas, a smooth gradient pulls the robot toward the desired exit

**How it works at an intersection:**
```
Global plan says: "Turn RIGHT at next intersection"

                             ← West branch (not our direction)
                              |
North branch (not) ──────────●────────── East branch (not)
                              |
                   South (where we came from)
                              |
                           [ROBOT]

Guidance field: A gentle cost gradient sloping toward the RIGHT branch.
A* naturally routes through the rightward exit.
No need to classify "which right" — the gradient resolves ambiguity.
```

**How it works in a plaza:**
```
Plaza (open concrete, fully drivable)
  ┌─────────────────────────────────────┐
  │                                     │
  │     ROBOT → → → → → → → → → → → →  │  ← Guidance gradient
  │                                     │     points to exit
  │                                     │
  │                              ┌──────┤
  │                              │ Exit │  ← Destination
  │                              └──────┤
  └─────────────────────────────────────┘

This is open space. Corridor-following fails.
But costmap + guidance field = robot crosses plaza toward exit,
naturally avoiding obstacles on the way.
```

#### Layer 4 (optional): Temporal Cost
**Source:** History of obstacle positions

- Cells where obstacles were recently seen get decayed cost (remember a pedestrian passed through here)
- Smooths out oscillations from dynamic obstacles

### Why This Avoids If-Else

Every scenario becomes a **configuration of the same cost layers:**

| Scenario | Drivable Layer | Obstacle Layer | Guidance Field |
|---|---|---|---|
| Straight corridor | Sidewalk=0, grass=255 | Empty | Forward bias |
| Intersection | Sidewalk=0, grass=255 | Empty | Toward correct branch |
| Plaza crossing | Plaza=0, grass=255 | Pedestrians | Toward exit |
| Obstacle on path | Sidewalk=0, grass=200 | Pedestrian cell=255 | Forward bias |
| Crosswalk | Crosswalk=0, road=50 | Cars | Toward crosswalk exit |

The A* planner doesn't know what scenario it's in. It just finds the lowest-cost path through the grid. The layers guide it naturally.

### Intersection Disambiguation (The Hard Part)

For the guidance field to work, the robot must know **which physical direction** corresponds to "right" from the global plan. This requires:

1. **Branch detection:** When the robot approaches an intersection, the segmentation will show multiple direction branches (the sidewalk widens and splits). Detect these as distinct "corridors" radiating from the junction.

2. **Branch-to-command mapping:** The global plan says "turn right." The robot sees 3 branches (forward, left, right). The branch with heading ~-90° from current heading is "right." In a 5-way with two rightward branches, pick the one closest to -90°.

3. **Fallback:** If the robot can't resolve which branch (e.g., occlusion), slow down and use the guidance gradient to smoothly navigate until the branch becomes clear.

**Branch detection algorithm (geometric, not ML):**
- At each step, analyze the contour of the drivable region
- Compute the **medial axis** (or distance transform) of the drivable region
- Where the medial axis splits → that's an intersection
- Classify each branch by direction angle from center
- Match to expected branch from global plan

Actually, the costmap approach makes branch detection **optional**, not required:

- If the robot can detect branches and map them to commands: guidance field is strong (certain)
- If the robot cannot resolve branches (e.g., too far from intersection, bad lighting): guidance field is weak (just a gentle forward bias), and the robot continues until the intersection resolves visually

This graceful degradation is the key robustness property.

---

## 4. Implementation Plan (Phased)

### Phase 0: Simulation Foundations ✅ (Done)
- Working Gazebo simulation with hambot model
- Sidewalk world with 4-way intersection
- Segmentation camera → Sidewalk Segmenter node
- Simple centroid navigator (baseline)

### Phase 1: Costmap Infrastructure
**Goal:** Replace centroid navigator with a DIY costmap + A* planner, running on the same segmentation mask. Straight sidewalk only.

1. **Implement `local_costmap_planner.py`:**
   - Maintain a 2D costmap grid (e.g., 3×3 m, 5 cm resolution = 60×60 cells)
   - Aligned to robot `base_link` frame (robot always at center-bottom of grid)
   - Project segmentation mask pixels to ground plane using camera model + TF
   - Mark sidewalk cells as cost 0, non-sidewalk as 255
   - Inflate edges with gradient cost
   - Run A* from robot position to a goal point ~2 m ahead along center of visible sidewalk
   - Extract steering angle from path, publish cmd_vel

2. **Validate in simulation:**
   - Robot drives straight down sidewalk autonomously
   - Recovers if pushed off-center
   - Handles curves (simulation only has straight, but can manually test)

### Phase 2: Obstacle Layer (LiDAR Integration)
**Goal:** Fuse LiDAR data into costmap for obstacle avoidance.

1. Subscribe to `/scan` (from simulated or real LiDAR)
2. Project laser points into costmap cells
3. Mark as lethal (255), inflate by robot radius
4. A* now automatically routes around obstacles
5. If sidewalk is fully blocked, planner may choose to briefly drive on grass (cost 200 vs 255 for staying on blocked sidewalk)

### Phase 3: Guidance Field + Intersections
**Goal:** Handle intersections using the global guidance layer.

1. Extend costmap to include guidance layer
2. At intersections (detected when drivable region widens significantly):
   - Compute a "preferred direction" based on a simple test command (hardcode for now)
   - Apply gradient cost biasing that direction
3. A* naturally routes through the correct branch

### Phase 4: Semantic Branch Detection
**Goal:** Improve intersection handling with proper branch-to-command mapping.

1. Analyze drivable contour at junctions
2. Extract branch directions
3. Map "turn right" → specific branch heading
4. Apply stronger guidance bias to that branch

### Phase 5: Plaza Handling
**Goal:** Navigate open plazas.

1. Add a "plaza mode" where the drivable layer covers a large area uniformly
2. Guidance field provides the only direction signal (gentle gradient toward destination)
3. Obstacle layer still works (pedestrians, benches)
4. Costmap naturally routes around obstacles while progressing toward goal

### Phase 6: Global Planner Integration
**Goal:** Connect to a global route planner.

1. Global planner produces a sequence of waypoints + semantic tags
2. Semantic Guidance Layer interprets these for the local costmap
3. Robot follows route end-to-end

### Phase 7: Real-World Deployment
**Goal:** Run on physical RPi + OAK-D.

1. Replace simulated segmentation camera with OAK-D on-device segmentation model
2. Run costmap planner on RPi (Python, lightweight)
3. Tune parameters for real-world conditions

---

## 5. Why Not Nav2?

Nav2 (ROS 2 Navigation2) is the standard solution for indoor robot navigation. Here's why it's not the right fit for this project:

| Requirement | Nav2 Assumption | Campus Reality |
|---|---|---|
| Mapping | Pre-built 2D occupancy grid (SLAM) | No detailed map available. Could build one, but campus changes (construction, events). Rough topology is more practical. |
| Localization | Accurate pose in map (AMCL) | LiDAR-only localization on symmetric sidewalks is fragile. Visual odometry + semantic landmarks more reliable. |
| Costmap source | LaserScan → obstacles | We need **semantic terrain classification** (is this grass or sidewalk?), not just geometric obstacles. |
| Path representation | Continuous geometric path | Paths are better expressed as "follow sidewalk then turn right at intersection" — topological, not just geometric. |
| Recovery behaviors | Rotate, back up, clear costmap | Need campus-specific recovery (e.g., stop for crossing pedestrians, briefly go on grass to bypass) |
| Compute | Desktop-class | Pi 4B 2GB. Nav2 is ~15+ nodes running simultaneously. A single integrated costmap node is lighter. |

Nav2 could still be used as the **local planner** (DWB controller) with a custom costmap plugin. But a DIY approach gives more control and is more educational.

---

## 6. Key Design Decisions

### Decision 1: Grid Resolution
- **5 cm cells** (3 m × 3 m = 60×60 = 3600 cells)
- Small enough to navigate narrow sidewalks (1.2 m = 24 cells)
- Large enough for A* to run in <10 ms on RPi
- Memory: ~3.6 KB per cost layer (trivial)

### Decision 2: Costmap Frame
- **`base_link`** (robot-centered), not `odom`
- Grid always centered on robot, aligned to forward direction
- No need for odometry to fill the grid — pure sensor data in robot frame
- Global planner outputs are transformed into robot frame at each tick

### Decision 3: Path Planning Horizon
- **3-5 m ahead** (what the camera can see + LiDAR)
- Replan every cycle (~10 Hz)
- At 0.25 m/s, robot moves 2.5 cm between replans (half a cell)
- If no valid forward path exists: stop, rotate slowly to scan, try again

### Decision 4: Control Law
- **Simple:** Extract first ~1 m of A* path, compute bearing to that point
- **Alternative:** Pure Pursuit on the full path (smoother, handles curves)
- Both output cmd_vel

### Decision 5: Semantic Segmentation Model
- **Simulation:** Gazebo segmentation camera (ground truth labels)
- **Real OAK-D:** Run lightweight model (e.g., MobileNetV3-Seg, PIDNet-S) on OAK-D VPU
- **Fallback:** Color-based segmentation (green=grass, grey=concrete) for proof of concept on real hardware

---

## 7. Open Questions

1. **Crosswalks:** How to detect and handle road crossings? Stop and wait for no traffic? Use OAK-D depth to detect approaching vehicles?

2. **Curb ramps:** Are they detectable in segmentation? They're transitions from sidewalk to road. Need special handling.

3. **Elevation changes:** Campus has hills. The 2D costmap assumes flat ground. Do we need a 2.5D costmap (with height)?

4. **Occlusions:** A pedestrian blocks the camera's view of the sidewalk. The robot can't see where the path goes. Rely on LiDAR + momentum?

5. **Global planner format:** Should it output (x, y) waypoints in a map frame? Or higher-level semantic commands? The latter is more robust but harder to compute.

6. **GPS integration:** Even noisy GPS could help disambiguate which intersection branch to take. Is it worth adding?

---

## 8. Glossary

| Term | Meaning |
|---|---|
| **Costmap** | 2D grid where each cell has a cost (0=free, 255=lethal). Path planner finds lowest-cost route through this grid. |
| **A*** | Graph search algorithm. Guarantees shortest path in grid if heuristic is admissible. |
| **Guidance Field** | A cost layer that encodes global intention — a gentle gradient directing the robot toward its goal. |
| **Semantic Segmentation** | Per-pixel classification of camera image (sidewalk, grass, building, person, etc.). |
| **Plaza** | Open concrete area without defined path boundaries. Common on USF campus (e.g., MSC plaza). |
| **Branch Detection** | Identifying discrete corridor directions at a sidewalk junction. |
| **Topological Path** | A route described as operations (follow, turn) rather than continuous (x,y) coordinates. |
| **DWA / TEB** | Local planners from Nav2. DWA = Dynamic Window Approach. TEB = Timed Elastic Band. |
| **OAK-D** | OpenCV AI Kit with Depth. Has neural network inference accelerator (VPU). |
