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
