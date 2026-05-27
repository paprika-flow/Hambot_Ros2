[//]: # (Output from Gemini 3.5 Flash)

To get ROS 2 running on your physical robot, we have to navigate a specific constraint: **your Raspberry Pi 4 (2GB RAM) is running Debian 12 (Bookworm)**. 

Because official ROS 2 binary packages are only pre-compiled for Ubuntu, there are no official `.deb` files for Debian. Furthermore, attempting to compile ROS 2 from source on a 2GB RAM Pi 4 will run the system out of memory (OOM) and cause it to crash.

To solve this, the industry-standard approach is to **run a headless, lightweight ROS 2 Docker container directly on the Raspberry Pi**. This allows you to install official, pre-compiled ROS 2 binaries inside an Ubuntu-based container while retaining full, direct access to the Pi's physical hardware (USB ports, Serial, and GPIO).

Here are the step-by-step instructions to initialize ROS 2 on your physical robot.

---

### Step 1: Install Docker on the Raspberry Pi 4 (Debian Host)
SSH into your Raspberry Pi and install Docker.

```bash
# Update the Debian host system
sudo apt update && sudo apt upgrade -y

# Install Docker using the official convenience script
curl -sSL https://get.docker.com | sh

# Add your current user to the docker and dialout groups 
# (This allows you to run Docker and access USB/Serial ports without root privileges)
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER

# Apply the group changes without rebooting
newgrp docker
newgrp dialout
```

---

### Step 2: Configure USB and Serial Rules (udev) on the Pi Host
The underlying Debian host needs to allow read/write access to the connected USB LiDAR and OAK-D camera so the Docker container can communicate with them.

1. **Create a udev rule for the OAK-D Lite camera:**
   ```bash
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
   ```

2. **Create a udev rule for the USB LiDAR (assuming a USB-to-UART CP210x chip like RPLIDAR):**
   ```bash
   echo 'KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-serial.rules
   ```

3. **Reload and trigger the rules:**
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

---

### Step 3: Create the ROS 2 Hardware Workspace on the Pi
Create a dedicated folder on the Pi where your custom ROS 2 nodes will live.
```bash
mkdir -p ~/robot_ws/src
```

---

### Step 4: Write the Docker Compose Configuration
By using Docker Compose, we can map your Pi's hardware directories directly into a lightweight, headless **ROS 2 Humble** container.

Create a `docker-compose.yml` file in `~/robot_ws/`:
```bash
nano ~/robot_ws/docker-compose.yml
```

Paste the following configuration:
```yaml
services:
  ros_robot:
    image: ros:humble-ros-base
    container_name: physical_robot
    restart: always
    network_mode: host
    ipc: host
    privileged: true  # Grants container access to GPIO, USB, and Serial ports
    volumes:
      - /dev:/dev
      - /sys:/sys
      - ./src:/workspace/src
    environment:
      - ROS_DOMAIN_ID=30 # Change this to any ID (0-232) to keep traffic isolated to your robot
    working_dir: /workspace
    command: sleep infinity
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

---

### Step 5: Start the ROS 2 Container
Launch the container. It will run in the background.
```bash
cd ~/robot_ws
docker compose up -d
```

Verify that the container is running:
```bash
docker ps
```

---

### Step 6: Test Hardware Access Inside Docker
Plug in your LiDAR and your OAK-D Lite to the Pi's USB ports (use the blue USB 3.0 ports for the camera to ensure enough bandwidth). Enter the container and check if the hardware is detected:

```bash
# Open a terminal shell inside your running container
docker exec -it physical_robot bash

# Check if the OAK-D Lite (03e7:f601 or similar Luxonis ID) is listed
lsusb

# Check if the LiDAR (usually /dev/ttyUSB0 or /dev/ttyACM0) is listed
ls -la /dev/ttyUSB*
```

---

### Step 7: Install and Compile Drivers Inside the Container

While remaining inside your interactive container shell, configure your workspace and install the driver binaries.

1. **Install dependencies and setup tools:**
   ```bash
   apt update && apt install -y git python3-colcon-common-extensions python3-rosdep usbutils
   rosdep update
   ```

2. **Install the OAK-D Lite ROS 2 Driver:**
   Because your Docker container runs Ubuntu 22.04, you can directly install pre-built binary packages instead of compiling the massive camera driver from source:
   ```bash
   apt update && apt install -y ros-humble-depthai-ros-driver
   ```

3. **Download and Build the LiDAR Driver (Assuming RPLIDAR):**
   ```bash
   cd /workspace/src
   git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   ```

4. **Build the Workspace:**
   On a 2GB RAM Pi, running standard compilation will overwhelm the memory and crash the system. **Limit colcon to a single thread** during compilation:
   ```bash
   cd /workspace
   colcon build --symlink-install --parallel-workers 1
   ```

---

### Step 8: Start the Physical Sensors

Now you can run the drivers to publish the active sensor topics.

* **Terminal 1 (Start the LiDAR):**
  ```bash
  # Access the container if you opened a new terminal
  docker exec -it physical_robot bash
  
  # Source the built workspace
  source /workspace/install/setup.bash
  
  # Launch the LiDAR (Replace rplidar_a1_launch.py with your specific LiDAR's model launch file if needed)
  ros2 launch rplidar_ros rplidar_a1_launch.py
  ```

* **Terminal 2 (Start the OAK-D Lite):**
  SSH into the Pi again, open a second shell inside the container, and run:
  ```bash
  docker exec -it physical_robot bash
  source /opt/ros/humble/setup.bash
  
  # Launch the camera node
  ros2 launch depthai_ros_driver camera.launch.py
  ```

---

### Step 9: View the Live Data on Your Mac (or PC)

Because ROS 2 uses DDS to automatically discover nodes across a local network:
1. Connect both your Pi and your Mac/PC to the same local lab Wi-Fi network.
2. Ensure you have set the exact same `ROS_DOMAIN_ID` on both machines (we used `30` in step 4).
3. If you run `ros2 topic list` on your Mac's ROS 2 terminal, the physical sensor topics `/scan` and `/oak/rgb/image_raw` will automatically appear. You can visualize them using RViz2 running locally on your laptop while the physical processing stays on the Pi.