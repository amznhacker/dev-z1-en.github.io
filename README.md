## System Requirements

- **Operating System**: Ubuntu 18.04 (ROS Melodic) or Ubuntu 20.04 (ROS Noetic, recommended).  
- **ROS**: Desktop-Full installation required.  
- **Dependencies**: `libboost-dev`, `libeigen3-dev`, and other packages installed via `rosdep`.  
- **Hardware**:  
  - PC with at least Intel i5/i7 or equivalent.  
  - Ethernet port for direct connection to the Z1 controller.  

---

## Installation Workflow

1. **On the Z1 Controller**  
   - Runs the `z1_controller` service for hardware communication.  

2. **On the Development PC (Ubuntu)**  
   - Install `z1_sdk` to send commands.  
   - Install `unitree_ros` and `unitree_legged_msgs` to use ROS interfaces.  
   - Build all packages in a `catkin_ws`.  

---

## Simulation vs Real Hardware

- **Simulation** (Gazebo + RViz)  
  - Safest environment to develop algorithms.  
  - Run `roslaunch unitree_ros z1_gazebo.launch`.  

- **Real Hardware**  
  - Connect via Ethernet (default IP `192.168.123.xxx`).  
  - Verify network setup with `ping`.  
  - Launch controller with `roslaunch unitree_ros z1_control.launch`.  

---

## Typical Development Workflow

1. Start `roscore`.  
2. Run simulation or connect to hardware.  
3. Send commands using SDK API (C++/Python).  
4. Monitor feedback topics (e.g. `/z1/joint_states`).  
5. Iterate with your custom control algorithms.  

---

## Troubleshooting

- **Cannot connect to arm** → Check Ethernet IP configuration.  
- **ROS node fails** → Ensure `roscore` is running and sourced (`source ~/catkin_ws/devel/setup.bash`).  
- **Arm not moving** → Verify power is on, E-stop released, and commissioning completed.

---

## Operating Procedures

To ensure safe and reliable use of the Unitree Z1 arm, follow these steps before, during, and after operation:

### Before Operation
1. **Workspace Check**
   - Ensure the area is free of people, pets, and obstacles.
   - Confirm the arm is securely mounted on a stable surface.
   - Verify that all cables (power, Ethernet) are firmly connected and not under tension.

2. **System Initialization**
   - Power on the Z1 controller and confirm indicator LEDs are normal.
   - On the PC, ensure `roscore` is running before launching any control nodes.
   - Run the system in simulation first to validate new programs.

3. **Safety Readiness**
   - Confirm an **E-stop button** (hardware or software) is available and functional.
   - Set speed/torque limits in the control software before executing motions.

### During Operation
1. Start with **low-speed movements** when testing.
2. Always monitor the arm visually — do not leave it unattended.
3. Keep hands, face, and objects away from the moving arm.
4. Immediately press **E-stop** if unexpected motion occurs.

### After Operation
1. Stop the control program before powering down the arm.
2. Power off the Z1 controller and disconnect power if storing the unit.
3. Coil cables neatly and ensure connectors are not strained.
4. Log any irregularities (unusual noise, vibrations, overheating) for maintenance.

---

## Maintenance Guidelines

Proper maintenance ensures long-term reliability and safety:

### Daily / After Each Use
- Wipe the arm and joints with a **dry, lint-free cloth** (never use water or solvents).
- Inspect power and communication cables for fraying or loose connections.
- Ensure ventilation around the controller is unobstructed.

### Weekly
- Check for **loose screws or mounting bolts**; tighten if needed.
- Test the **E-stop** to ensure it functions properly.
- Inspect the arm for dust accumulation; use compressed air if necessary.

### Monthly
- Review software and firmware updates for the SDK and controller.
- Inspect joints for signs of **abnormal wear** or overheating.
- Verify smooth manual motion (when powered off) to detect resistance or misalignment.

### Long-Term Storage
- Store in a **dry, dust-free environment** between 10–30°C.
- Cover with a protective cloth or enclosure.
- Disconnect and safely coil cables to avoid bending or pinching.

---

⚠️ **Important**: Only qualified personnel should perform in-depth repairs or firmware flashing.  
Routine users should restrict maintenance to inspection, cleaning, and basic checks.

