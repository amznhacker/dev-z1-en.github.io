# 🧠 Robot & Embedded Device Network Setup Cheat Sheet

## ⚙️ The Five-Layer Checklist

### 1. Physical Layer — Check Link
Verify cable and port:
```bash
sudo ethtool enp61s0 | grep Link
```
✅ Should show: `Link detected: yes`  
If not → replace cable, port, or adapter.

---

### 2. DHCP Test — Who’s Serving IPs?
Check if a DHCP server replies:
```bash
sudo dhclient -v enp61s0
```
- If you see **DHCPOFFER**, it’s a DHCP world (router / managed switch).  
- If **no offers**, it’s a static IP device (robot, camera, LiDAR, etc.).

---

### 3. Find or Guess Default Static IP Range
Common defaults:
| Vendor / Type | Typical IP | Notes |
|----------------|-------------|-------|
| Unitree Z1 / Go2 | 192.168.123.110 | PC must be 192.168.123.xxx |
| Intel RealSense | 192.168.15.1 | Fixed subnet |
| PLCs, LiDARs | 192.168.0.x or 10.0.0.x | Industrial defaults |

If unsure, scan:
```bash
sudo nmap -sn 192.168.0.0/24
sudo nmap -sn 192.168.123.0/24
```

---

### 4. Assign Yourself a Static IP
Match your PC to the device subnet:
```bash
sudo nmcli con add type ethernet ifname enp61s0 con-name z1   ipv4.method manual ipv4.addresses 192.168.123.162/24 ipv6.method ignore
sudo nmcli con up z1
```
Then test:
```bash
ping 192.168.123.110
```

---

### 5. Application Layer — Control It
Access via SSH or SDK:
```bash
ssh unitree@192.168.123.110
# password: 123

cd ~/unitree_arm_sdk
./z1_ctrl
```

---

## 🧭 Quick Table — DHCP vs Static

| Feature | DHCP | Static |
|----------|-------|--------|
| IP Assigned Automatically | ✅ | ❌ |
| Common in | Home routers, laptops | Robots, PLCs, sensors |
| Needs Manual Config | ❌ | ✅ |
| How to Connect | Plug and play | Set static IP in same subnet |

---

## 🧩 Make It Persistent
Ensure connection auto-starts on reboot:
```bash
sudo nmcli con mod z1 connection.autoconnect yes
```
Or manually edit `/etc/NetworkManager/system-connections/z1.nmconnection`:
```
[ipv4]
method=manual
addresses1=192.168.123.162/24;
```

---

### 🛡️ Safety Reminders (for Unitree Z1)
- Zero position before power-on (J1 & J6 alignment).  
- Do **not** power with `z1_ctrl` already running.  
- Insulate unused 24V wires at the end effector.  
- Avoid singular positions after startup.

---

Made for **Unitree Z1 / Merced Robotics Lab**  
By: Ramiro González 🦾
