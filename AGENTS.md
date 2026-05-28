# Hambot Project: Hardware, OS, and Simulation Profile

### 1. Physical Robot Platform (Hambot)
* **Single Board Computer (SBC):** Raspberry Pi 4B (2GB RAM) running headless Debian 12 (Bookworm).
* **Sensors:** 2D USB LiDAR (Slamtec) and an OAK-D Lite depth camera.
* **ROS 2 Version:** ROS 2 Humble Hawksbill (containerized on Ubuntu 22.04).
* **ROS 2 Middleware:** CycloneDDS (`rmw_cyclonedds_cpp`) running on `ROS_DOMAIN_ID=30`.

### 2. Simulation and Development Host
* **Hardware:** MacBook Air (M2, 16GB RAM) running macOS.
* **Simulation Workspace Directory:** Located on the Mac host at `~/hambot_sim_ws/`.
* **Execution Environment:** Runs an ARM64 Ubuntu container with a VNC desktop accessible via browser at `http://localhost:6080/vnc.html`.
* **Rendering Configuration:** Uses Mesa Software Rendering (`LIBGL_ALWAYS_SOFTWARE=1`) to bypass Apple Silicon OpenGL virtual device rendering bugs.

### 3. Robot Mechanical Properties (URDF Configured)
* **Chassis Dimensions:** Length: 193 mm (`0.193`), Width: 157 mm (`0.157`), Height: 100 mm (`0.100`), Mass: 2.5 kg.
* **Wheel Dimensions:** Radius: 45 mm (`0.045`), Thickness: 10 mm (`0.010`), Mass: 0.15 kg.
* **Wheel Placement:** x-offset: 51 mm, y-offset: 99.5 mm (16 mm clearance from chassis side). Wheel separation is derived dynamically as `wheel_joint_y * 2` (199 mm).
* **Ground Clearance / Base Offset:** 71 mm ground clearance offset (`base_link_z_offset = 0.071`).
* **Sensor Mounting:** 
  * LiDAR (`laser_frame`) positioned in the exact center of the chassis, visual cylinder origin offset up by 20 mm to sit on top of the chassis.
  * Camera support mast standing 250 mm tall mounted at the rear of the chassis (`X = -0.0915`).
  * Camera link (`camera_frame`) mounted 220 mm up the support mast, offset 12.5 mm forward to sit flush on the front face of the stand.

### 4. Simulation World Profile (`campus_sidewalk.sdf`)
* **Grass Plane (Label 2):** Flat green plane, top surface sitting at Z = 0.0. Visual tag carries semantic label `2` via the `ignition-gazebo-label-system` plugin.
* **Sidewalk Network (Label 1):** Grey sidewalk network, top surface raised to Z = 0.02 (creating a physical 2 cm curb lip). Visual tags carry semantic label `1`. Includes a 5 m South straightway, a 1.2 m center intersection, and 3 m North, East, and West branching sidewalks.