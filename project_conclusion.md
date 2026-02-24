# Skypal Project Conclusion: Hardware-in-the-Loop Integration

## What We Accomplished
We successfully designed, built, and launched a complex **Hardware-in-the-Loop (HITL) Simulation Architecture** that bridges a simulated PX4 drone in Gazebo with real-world, distributed ROS 2 hardware.

1. **Custom VPN & RJ45 Routing**: We proved that a Raspberry Pi (`skyberry`) can control a high-fidelity PC simulation by separating the network streams. We routed all ROS 2 topics over a Tailscale VPN (emulating real-world cellular latency) while locking the PX4 Micro XRCE-DDS Agent connection to a direct RJ45 ethernet wire to prevent the PX4 simulation from crashing under the VPN latency.
2. **Failsafe Heartbeat System**: We created a `skypal_core` node running directly on the Pi that constantly measures the VPN latency to the PC's `skypal_controller`. We successfully tested breaking the connection; the Pi instantly overrode the user's flight commands with zero-velocity Setpoints, forcing the Gazebo drone to freeze and hover safely.
3. **DualShock 4 Integration**: We mapped a DualShock controller into proper standard RC Mode 2 commands, successfully bridging the joystick inputs across the VPN into live Offboard velocity setpoints.
4. **Mission Control Tkinter GUI**: We built a custom desktop application to orchestrate the 6 distributed terminal processes, complete with staggered boot sequences, IP parameterization, and dynamic Gazebo world selection.

## Hurdles Overcome
1. **DDS Discovery Isolation**: Initially, the ROS 2 nodes on the PC and Pi could not see each other over Tailscale. We overcame this by launching a dedicated `fastdds` Discovery Server, explicitly binding the nodes to the VPN IP (`100.122.20.128:11811`), and overriding the default multicast.
2. **PX4 Offboard Mode Rejection**: Even when connected, PX4 originally refused to enter Offboard Mode. We discovered PX4 requires a continuous, active stream of `OffboardControlMode` *before* it allows the state switch. We also discovered PX4 inherently rejects Setpoints unless unused values (like acceleration, jerk, and yaw) are explicitly cast to `NaN` (Not a Number). Modifying the Python node fixed this.
3. **Coordinate Frame Twist**: The drone flew sideways when commanded forward. We solved this physics mismatch by updating the Python node to subscribe to the drone's IMU Quaternions, calculating its live Yaw, and dynamically translating the Body-Fixed joystick commands into the Earth-Fixed (NED) vectors that PX4 requires.

---

# Next Phase: Autonomous Flight Architecture

## Your Objective
Deploying a Computer Vision model (like MobileNetV2) onto the Raspberry Pi to collect data and eventually drive the drone autonomously without the DualShock.

## Evaluating the Approach: MobileNetV2 on Raspberry Pi
Your plan to use MobileNetV2 is **excellent** and conceptually sound, but depends heavily on execution.

**Why MobileNetV2 is a good choice:**
* It is specifically architected for edge devices. Its deep depth-wise separable convolutions drastically reduce the parameter count, meaning it can run inference directly on a Raspberry Pi CPU/GPU without needing a heavy desktop graphics card.
* It is extremely fast. For real-time drone control, inference speed (FPS) is far more important than raw classification accuracy. If the drone is moving at 5 m/s, a 500ms delay in object detection means the drone travels 2.5 meters blind. MobileNetV2 minimizes this latency.

**The Challenges (And How You Should Address Them):**
1. **The Pi's Computing Bottleneck:** While the Pi 4 is capable, running a ROS 2 DDS Server, the Skypal Core flight node, *and* a continuous neural network inference loop on the CPU will quickly max out its thermal limits. 
   * **Recommendation:** You should not run standard TensorFlow on the Pi. You should convert your MobileNetV2 model to **TensorFlow Lite (TFLite)**. TFLite is infinitely more efficient on ARM processors. Alternatively, consider a USB Coral Edge TPU accelerator to offload the vision processing entirely.
2. **Data Collection Strategy:** Your approach to collect data in Gazebo first, then real life, is standard industry practice (Sim2Real). However, Gazebo lighting and textures are perfect. Real life has glaring sun, shadows, and motion blur.
   * **Recommendation:** When training your model, heavily apply "Data Augmentation" to your Gazebo screenshots (randomly altering brightness, adding Gaussian noise, blurring the images). This will force the MobileNet model to learn the shape of the objects, rather than memorizing the perfect colors of the Gazebo simulation.

## The Roadmap
1. **Add Camera to Gazebo:** Modify the `x500` SDF model to attach a simulated downward or forward-facing camera plugin.
2. **Bridge Video to ROS 2:** Use the `ros_gz_image` bridge to stream the Gazebo camera pictures into a `sensor_msgs/Image` ROS 2 topic on the Raspberry Pi.
3. **Data Collection Node:** Write a simple Python script on the Pi that subscribes to the Image topic and saves pictures to an `images/` folder whenever you press a button on your DualShock 4. Fly around the Baylands world and press the button to build your dataset!
4. **Train and Deploy:** Train MobileNetV2 on your PC using the saved images, convert the `.h5` model to a `.tflite` file, and `scp` it back to the Pi.
5. **Autonomy Node:** Write a new ROS 2 node on the Pi that runs the TFLite inference in real-time. Instead of the `skypal_controller`, this new node will publish velocities directly to the `heart_node` to steer the drone toward what it sees!
