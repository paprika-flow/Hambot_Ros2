Here is your step-by-step simulation development workflow. This is designed so you can **write code comfortably on your Mac host**, but **execute compiles and run 3D simulations inside your VNC container** where the graphics are handled safely.

---

### 1. Start of Session

1. **Start the environment on your Mac host terminal:**
   ```bash
   cd ~/hambot_sim_ws
   docker compose up -d
   ```
2. **Open the graphical interface:**
   Go to 
   ```bash
   http://localhost:6080/vnc.html
   ``` 
3. **Open the code editor on your Mac host:**
   Open VS Code (or your preferred editor) on your Mac and open the local `~/hambot_sim_ws` folder. You will write all your robot's Xacro/URDF and Python code here.

---

### 2. Compiling and Running the Simulation
To build and launch:

1. **Compile the workspace:**
   Inside VNC browser window, open a **Terminal** (we will call this **VNC Terminal 1**) and run:
   ```bash
   cd ~/hambot_sim_ws
   colcon build --symlink-install
   ```
2. **Launch the Gazebo simulation:**
   In **VNC Terminal 1**, source the newly compiled workspace and launch simulation (which loads Gazebo and spawns the robot):
   ```bash
   source install/setup.bash
   ros2 launch hambot_bringup sim_bringup.launch.py
   ```
3. **Launch RViz2:**
   Open a second terminal inside VNC window (**VNC Terminal 2**) to launch your visualization:
   ```bash
   rviz2
   ```

---

### 3. Teleoperating (Driving) the Robot
To drive simulated robot on its virtual sidewalks to verify the physics and sensor data:

1. Open a third terminal inside your VNC window (**VNC Terminal 3**).
2. Run the keyboard control node:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Keep this terminal selected, use your keyboard keys (`u i o j k l m , .`) to drive, and watch your robot navigate the Gazebo world and update its laser scans in RViz2.

---

### 4. Adding a New Package (The Try-on-the-Fly Loop for Sim)
If you need a new ROS 2 tool or package (e.g., `ros-humble-joint-state-publisher-gui`) while designing your robot:

1. **Install it on the fly:**
   Inside any terminal in your VNC desktop, install the package using `sudo` (the default `ubuntu` user has passwordless sudo privileges in this image):
   ```bash
   sudo apt update && sudo apt install -y ros-humble-joint-state-publisher-gui
   ```
2. **Verify it works:** Run the newly installed node to ensure it functions as expected.
3. **Save the recipe:** Open the `Dockerfile` inside `~/hambot_sim_ws/` on your Mac host and append `ros-humble-joint-state-publisher-gui \` to the list of packages.
4. **Keep working:** Do not rebuild yet. The temporary installation keeps your current workspace active.

---

### 5. End of Session (Baking Your Changes)
When you are ready to stop for the day you can stop the container.

If you made any changes lock in any new packages you added to your `Dockerfile`:

1. Close the running terminals inside your VNC window.
2. In your Mac's host terminal (outside of VNC), run:
   ```bash
   cd ~/hambot_sim_ws
   docker compose down
   
   # Run the custom build script to increment your version and bake the changes if you made any changes
   ./build.sh
   ```
3. Type your new version number (e.g., `1.1.0`) and press **`Enter`**.
4. To verify your new container is built and running with the new packages permanently saved:
   ```bash
   docker compose up -d
   ```