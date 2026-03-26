# SkyPal System Architecture & Development History

**Purpose of this Document**
This `.md` file is a permanent architectural reference guide for developers and future AI agents. It tracks the complete composition, major milestones, and orchestration logic of the SkyPal Drone Delivery pipeline we have built so far. Agents should use this file to quickly understand node multiplexing, the Gazebo-to-Real-Flight transition logic, and GUI integration constraints before making modifications.

---

## 1. Operating Modes & The Base Station GUI (`px4_sim_launcher.py`)
The system orbits around a central Python Tkinter Application (`px4_sim_launcher.py`). This GUI acts as a master sequence dispatcher and process manager for 11 individual interconnected components.

**Flight Modes Supported:**
* **PURE_SIM**: Pure software Gazebo SITL simulation. All nodes run locally; explicitly passes `export ROS_DISCOVERY_SERVER=127.0.0.1:11811`.
* **RPI_SITL**: Hardware-in-the-Loop simulation where critical logic (`heart_node`, `qr_scanner`) executes remotely on the Raspberry Pi ("Skyberry") over a Tailscale VPN, but the Gazebo physics engine renders locally.
* **REAL_FLIGHT**: Physical flight. Hides simulated components (Gazebo, LiDAR bridge) and relies fully on real-world Pixhawk / V4L2 USB camera data.

**GUI Features:**
* **Dynamic Tab Management**: The GUI dynamically destroys and rebuilds the process terminal tabs (`self.notebook.forget()`) when switching modes to hide irrelevant nodes (e.g., hiding MAVProxy in simulation).
* **Live Mission Map**: Embeds a `tkintermapview` that tracks the drone in real-time. It runs a background thread executing `ros2 topic echo /fmu/out/vehicle_global_position` tied to the FastDDS server to draw a highly precise red breadcrumb path across the map.

---

## 2. Core ROS 2 Nodes (`skypal_core` & `skypal_controller`)

### `heart_node.py` (The Multiplexer)
The central nervous system linking the user's RC inputs to PX4.
* **Velocity Smoothing**: Implements an Exponential Moving Average (EMA) to filter out jitter over the mobile network (JIOPAL/AirFiber).
* **FLU to FRD Mapping**: Maps standard ROS 2 Twist logic (Forward, Left, Up) mathematically into PX4's Body frame (Forward, Right, Down) dynamically depending on the current Yaw.
* **Autonomy Yielding**: When an autonomous mission or Custom RTL is triggered, it releases `rc_override` and passively passes along positional logic from the `/skypal/autonomous_trajectory` topic straight to `/fmu/in/trajectory_setpoint`.
* **QoS Policies**: Crucially binds to `/fmu/out/vehicle_local_position` using `DurabilityPolicy.VOLATILE` (matching PX4's high-speed sensor durability) instead of `TRANSIENT_LOCAL`.

### `path_tracker_node.py` (Custom Trace-back RTL)
Replaces the default PX4 straight-line Return-To-Launch with a reverse trace trajectory.
* Subscribes to the vehicle's local position and caches an XYZ coordinate tuple every 1 meter of physical movement.
* Listens to `/skypal/sys_command`. Upon receiving `"nav_rtl"`, it switches to Playback mode.
* Publishes high-frequency `TrajectorySetpoint` arrays back to the `heart_node` multiplexer to smoothly back-fly waypoint-by-waypoint into the home origin.

### `mission_commander.py` (Cloud Firebase Bridge)
Handles macro-scale API integrations.
* Periodically polls the `skypal-app-6408` Firestore database for delivery missions.
* Uses a `PIVOT -> GLIDE -> LANDING -> AWAITING_SCAN` state machine to rotate the drone to the target bearing and push it horizontally to the designated GPS drop-off coordinates using custom WGS84-to-NED conversions.

### `controller_node.py` (Joystick Publisher)
* Processes Xbox/Playstation controller hardware inputs via `/joy` and packages them into `/skypal/cmd_vel` Twists. 
* Houses the emergency system command buttons configured to send messages like `"arm_offboard"` or `"nav_rtl"`.

### HITL Camera Nodes (`qr_scanner_node.py` & `camera_recorder_node.py`)
* Dedicated V4L2 computer vision processes designed to run on the Raspberry Pi edge.
* `qr_scanner_node.py`: Engages PyZbar over an OpenCV feed only when the drone is landed and disarmed to authorize cloud package drop-offs securely while sparing Pi CPU resources mid-flight.

---

## 3. Communication Architecture
* **FastDDS Discovery Server**: We strictly route all XRCE-DDS and internal ROS 2 chatter through a localized discovery server (Port `11811`). This prevents domain ID clashes and forces flawless VPN connectivity across Tailscale. 
* **Workspace Dependency**: All manual bash subprocesses (like buttons in the Tracker Map GUI) must strictly use `source ~/skypal_ws/install/setup.bash && export ROS_DISCOVERY_SERVER=...` simultaneously to decode the custom `px4_msgs` packet structs properly.
