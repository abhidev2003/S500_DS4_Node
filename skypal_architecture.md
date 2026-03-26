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
* **Live Mission Map & Interactive Planner**: Embeds a `tkintermapview`. It tracks the drone via `/fmu/out/vehicle_global_position` red breadcrumbs and allows operators to interactively map out flights by right-clicking on the map to spawn "Send (A)" and "Receive (B)" waypoints! Routing lines are projected globally across the map before the mission is committed to the local FastDDS network.

---

## 2. Core ROS 2 Nodes (`skypal_core` & `skypal_controller`)

### `heart_node.py` (The Multiplexer)
The central nervous system linking the user's RC inputs to PX4.
* **Velocity Smoothing**: Implements an Exponential Moving Average (EMA) to filter out jitter over the mobile network.
* **FLU to FRD Mapping**: Maps standard ROS 2 Twist logic mathematically into PX4's Body frame.
* **Autonomy Yielding**: When an autonomous mission or Custom RTL is triggered, it releases `rc_override` and passively passes along localized setpoints.

### `path_tracker_node.py` (Custom Trace-back RTL)
Replaces the default PX4 straight-line Return-To-Launch with a reverse trace trajectory.
* Bypasses the deactivated PX4 Local Position simulator topic by hijacking the global WGS84 GPS lock directly and mathematically translating coordinate distances independently.
* Operates an Omni-QoS Subscription strategy (`10`, `qos_profile_sensor_data`, and custom filters) to pierce through undocumented FastDDS Strict Capability isolation policies.
* Ends trace-backs with a forcefully manipulated `0.15 m/s` customized buttery-soft Offboard descent landing to simulate touch-down without cracking the physical carbon fiber frames.

### `mission_commander.py` (Autonomous Tactical Dispatcher)
Operates the fully decentralized Multi-Landing Sequence State Machine.
* Bypasses Firebase APIs for immediate local Base-Station tactical control by listening directly to `/skypal/local_mission`.
* Translates the payload coordinates sequentially across an internal execution array logic.
* **Multi-Landing Sequence**: Employs a complex transition pipeline (`PIVOT -> GLIDE -> LANDING -> AWAITING_TAKEOFF -> PIVOT`). Between waypoints, the drone forcefully commands standard `NAV_LAND` disarm physics dynamically at each stop off and holds the position rigorously for exactly 10 seconds before rocketing identically back into the Z-plane to service the next waypoint natively.

### `controller_node.py` (Joystick Publisher)
* Processes hardware inputs into twists. 
* Houses the emergency system command buttons configured to send messages like `"arm_offboard"` or `"nav_rtl"`.

### HITL Camera Nodes (`qr_scanner_node.py` & `camera_recorder_node.py`)
* Dedicated V4L2 computer vision processes designed to run on the Raspberry Pi edge.
* `qr_scanner_node.py`: Engages PyZbar over an OpenCV feed only when the drone is landed and disarmed to authorize payloads safely.

---

## 3. Communication Architecture
* **FastDDS Discovery Server**: We strictly route all XRCE-DDS and internal ROS 2 chatter through a localized discovery server (Port `11811`). This prevents domain ID clashes and forces flawless VPN connectivity across Tailscale. 
* **Workspace Dependency**: All manual bash subprocesses strictly use `source ~/skypal_ws/install/setup.bash && export ROS_DISCOVERY_SERVER=...` simultaneously to decode the custom `px4_msgs` packet structs properly.
