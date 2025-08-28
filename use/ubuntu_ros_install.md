# Ubuntu 20.04 + ROS Noetic Installation Guide (from Windows with Rufus)

## 1. Prepare Ubuntu ISO
- Download: **Ubuntu 20.04.6 Desktop (amd64)** from [ubuntu.com](https://ubuntu.com/download/desktop).

## 2. Create Bootable USB with Rufus
- Use a good USB drive (8 GB+).
- First USB was faulty (write-protected, I/O errors) → switched to a new one.
- Rufus settings:
  - Device: [USB drive]
  - Boot selection: `ubuntu-20.04.6-desktop-amd64.iso`
  - Partition scheme: **GPT**
  - Target system: **UEFI (non-CSM)**
  - File system: **FAT32**
- Click **Start**, choose *ISO mode* when asked.

## 3. Boot Into Ubuntu Installer
- Reboot → press **F12** (Alienware boot menu).
- Select USB (UEFI option).
- Choose **“Try or Install Ubuntu”**.

## 4. Fix Partitioning Issues
- Opened **GParted** from the live USB.
- Deleted all Windows partitions → whole disk empty/unallocated.

## 5. Install Ubuntu
- Ran installer, chose **Erase disk and install Ubuntu**.
- Created username, password, timezone.
- Installed and rebooted into Ubuntu 20.04.

## 6. Update System
```bash
sudo apt update && sudo apt upgrade -y
```

## 7. Install ROS Noetic
```bash
# Add ROS repo
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | \
  sudo tee /etc/apt/sources.list.d/ros1-latest.list
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
  sudo apt-key add -

sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Auto-source ROS
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dev tools & rosdep
sudo apt install -y python3-rosdep python3-rosinstall \
  python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

## 8. Install Dependencies
```bash
sudo apt install -y libboost-dev libeigen3-dev
```

## 9. Create Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 10. Verify ROS
```bash
rosversion -d     # expect: noetic
roscore           # should run without errors
```

## 11. Test with Turtlesim
- Terminal 1:
```bash
roscore
```
- Terminal 2:
```bash
rosrun turtlesim turtlesim_node
```
- Terminal 3:
```bash
rosrun turtlesim turtle_teleop_key
```
- Use arrow keys to drive the turtle.

---

# ✅ Final Result
- Clean Ubuntu 20.04 install (Windows fully removed).
- ROS Noetic + dependencies ready.
- Catkin workspace set up.
- Simulation works (`turtlesim`).
- Ready to add motion-control program and build.
