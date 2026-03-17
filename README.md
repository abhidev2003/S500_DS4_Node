# SkyPal ROS 2 DualShock 4 Mission Control

This repository contains the native ROS 2 packages and GUI launcher for controlling a PX4-based quadcopter over the internet using a DualShock 4 controller and Tailscale VPN. It bridges standard human interface devices to micro DDS flight controllers safely.

## Architecture & Node Communication Flow

The system operates on a dual-node architecture, totally bypassing heavy legacy tools like MAVROS in favor of high-speed Micro XRCE-DDS messaging.

### 1. `skypal_controller` (The Base Station Input Node)
This package runs locally on the Base Station laptop attached to the physical DualShock 4 controller.
*   **Subscribes to:** `/joy` (The raw Linux `/dev/input/js0` joystick axis and button array).
*   **Publishes:** 
    *   `/skypal/cmd_vel` (`geometry_msgs/Twist`): Scales raw stick inputs into X/Y/Z linear velocity requests and Yaw angular rates. It incorporates a **deadzone filter (15%)** to prevent resting stick drift from sending violent initialization spikes.
    *   `/skypal/sys_command` (`std_msgs/String`): Broadcasts high-level strings like `"arm_offboard"` or `"land"` based on discrete DS4 button presses (e.g., Options / Share).

### 2. `skypal_core` (The Flight Translation Node)
This node contains `heart_node.py` which can dynamically run *either* locally on the laptop (for Pure SITL) *or* natively on the Raspberry Pi companion computer (for Real Flights).
*   **Subscribes to:** `/skypal/cmd_vel` and `/skypal/sys_command` over the Tailscale VPN.
*   **Failsafe Monitor:** Analyzes the interval between incoming packets. If the connection drops for > 2.0 seconds, it instantaneously forces an `AUTO_HOVER` to prevent airborne flyaways.
*   **Publishes to PX4 (Micro XRCE-DDS):** 
    *   It mathematically translates the standard ROS `Twist` commands (Forward, Left, Up) into PX4's mandatory Local Geographic `NED` frame (North, East, Down).
    *   It pushes continuous `TrajectorySetpoint` messages to `/fmu/in/trajectory_setpoint` at 20Hz.
    *   It translates string system commands into `VehicleCommand` messages (e.g., `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `VEHICLE_CMD_DO_SET_MODE`) to seize offboard control immediately.
