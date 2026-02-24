# Skypal Hardware-in-the-Loop Architecture Workspace (`skypal_ws`)

This repository contains the **Hardware-in-the-Loop (HITL) Simulation Architecture** that integrates a physics-accurate UAV simulation (PX4 in Gazebo) with distributed physical hardware (Raspberry Pi) and a centralized Mission Control PC.

## Project Highlights
* **Custom VPN & RJ45 Routing Configuration**: A remote Raspberry Pi (`skyberry`) controls the desktop's high-fidelity simulation and teleoperation routing. Teleoperation and heartbeat telemetry are routed securely over a Tailscale VPN, establishing a reliable baseline for internet control latency. Concurrently, PX4 simulation traffic is routed synchronously via a direct RJ45 interface.
* **Robust Failsafe Heartbeat System**: Custom low-level `skypal_core` ROS 2 node physically monitors the latency of the VPN connection and gracefully forces a simulated flight hover immediately in the event of packet loss or excessive lag above 500ms.
* **DualShock 4 Teleoperation**: Complete custom mapping of a DualShock 4 controller over the VPN tunnel bridging virtual and real-world inputs.
* **Mission Control Multiplexer GUI**: Desktop Tkinter application handles automated boot sequences, dynamically configures IPs/ports, and allows single-click launch parameters directly into the Gazebo runtime.

## Core Setup Files
- `install_ros2.sh` / `install_ros2_pi.sh`: Scripts to quickly bootstrap the ROS 2 environment on the Mission Control machine and edge devices.
- `fastdds_setup.sh`: Script establishing custom discovery server configuration specifically optimized for bridging over Tailscale.
- `skypal_launcher.py`: Our core GUI that simplifies launching the whole system seamlessly across devices.

## Project Details
For a deeper dive into the system rationale, setup nuances, roadmap toward autonomous implementation (via MobileNetV2 dataset collection), and all hurdles tackled along the way, refer to [project_report.md](project_report.md) and [project_conclusion.md](project_conclusion.md) within this workspace.
