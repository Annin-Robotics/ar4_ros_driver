# AR4 ROS Driver

ROS 2 driver of the AR4 robot arm from [Annin Robotics](https://www.anninrobotics.com).
Tested with ROS 2 Jazzy on Ubuntu 24.04. Also has branch for Humble
[here](https://github.com/ycheng517/ar4_ros_driver/tree/humble).

**Supports:**

- AR4 MK1 (Original version), MK2, MK3, MK4
- AR4 servo gripper

**Features:**

- MoveIt control
- Gazebo simulation

## Video Demo

<div align="center">

|                                        Moveit Motion Planning                                         |                                   Startup, Calibration, and Gripper Control                                   |
| :---------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------: |
| [![AR4 ROS 2 Driver Demo](http://img.youtube.com/vi/XJCrfrW7jXE/0.jpg)](https://youtu.be/XJCrfrW7jXE) | [![Startup, Calibration, Gripper](http://img.youtube.com/vi/PQtXFzqRtHM/0.jpg)](https://youtu.be/PQtXFzqRtHM) |

</div>

## Add-on Features and Capabilities

The following projects showcases additional features and capabilities built on top of this driver:

- [Hand-Eye calibration](https://github.com/ycheng517/ar4_hand_eye_calibration)
- [Teleoperation using Xbox controller](https://github.com/ycheng517/ar4_ros_driver_examples)
- [Multi-arm control](https://github.com/ycheng517/ar4_ros_driver_examples)
- [Voice controlled pick and place](https://github.com/ycheng517/tabletop-handybot)

## Overview

- **annin_ar4_description**
  - Hardware description of arm & servo gripper urdf.
- **annin_ar4_driver**
  - ROS interfaces for the arm and servo gripper drivers, built on the ros2_control framework.
  - Manages joint offsets, limits and conversion between joint and actuator messages.
  - Handles communication with the microcontrollers.
- **annin_ar4_firmware**
  - Firmware for the Teensy and Arduino Nano microcontrollers.
- **annin_ar4_moveit_config**
  - MoveIt module for motion planning.
  - Controlling the arm and servo gripper through Rviz.
- **annin_ar4_gazebo**
  - Simulation on Gazebo.

## Installation (ROS 2 Jazzy – Ubuntu 24.04)

> 💡 **Recommendation**
> Before using ROS, it is strongly recommended that you first verify your AR4 robot works correctly using the official **AR4 Control Software** from Annin Robotics. This provides better tools for debugging mechanical, electrical, and calibration issues before introducing ROS.

This repository is developed and tested against **ROS 2 Jazzy on Ubuntu 24.04**. The installation process consists of several layers:

1. Install ROS 2 Jazzy (base system)
2. Create and build a ROS 2 workspace
3. Install ROS control and MoveIt
4. Clone and build the AR4 ROS driver
5. Configure serial permissions
6. Install Arduino + Teensy toolchain
7. Flash firmware to Teensy and Arduino Nano

The sections below walk through these steps in a clean, end‑to‑end order.

---

### 1. Install ROS 2 Jazzy

Follow the **official ROS 2 Jazzy installation guide for Ubuntu (Debian packages)**:

👉 [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Make sure that:

* You complete the **Environment setup** step
* You can run `ros2 --version` successfully

---

### 2. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

This creates a standard ROS 2 workspace layout:

```
ros2_ws/
 ├── src/
 ├── build/
 ├── install/
 └── log/
```

---

### 3. Clone the AR4 ROS Driver

```bash
git clone https://github.com/Annin-Robotics/ar4_ros_driver.git
```

Your workspace should now look like:

```
~/ros2_ws/src/ar4_ros_driver
```

---

### 4. Install ROS Dependencies

Initialize and update `rosdep` (only required once per system):

```bash
sudo rosdep init
rosdep update
```

Install all package dependencies for the workspace:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

### 5. Install ros2_control

The AR4 driver is built on **ros2_control**, which provides the hardware interface layer between MoveIt and the robot.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

---

### 6. Install MoveIt

MoveIt provides motion planning, collision checking, and trajectory execution.

```bash
sudo apt update
sudo apt install -y ros-jazzy-moveit
```

---

### 7. Build the Workspace

Source ROS and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build
```

Overlay the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

---

### 8. Automatically Source ROS (Recommended)

To avoid re‑sourcing ROS every terminal session, add the following lines to your `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Reload:

```bash
source ~/.bashrc
```

---

### 9. Verify Installation (MoveIt Demo)

Before connecting real hardware, verify that ROS + MoveIt are working:

```bash
ros2 launch annin_ar4_moveit_config demo.launch.py ar_model:=mk4
```

This should open RViz with the AR4 model and interactive planning tools.

---

### 10. Enable Serial Port Access

Required for communicating with the Teensy and Arduino Nano:

```bash
sudo adduser $USER dialout
```

Log out and back in (or reboot) for the change to take effect.

---

## Firmware Flashing

The AR4 uses **two microcontrollers**:

* **Teensy 4.1** – main motion controller
* **Arduino Nano** – servo gripper controller (if used)

Both must be flashed before running the ROS driver.

---

### 1. Install Arduino IDE (2.x)

#### Install FUSE (required for AppImage)

Ubuntu 24.04:

```bash
sudo apt update
sudo apt install -y libfuse2t64
```

#### Download and Run Arduino IDE

```bash
mkdir -p ~/apps/arduino
cd ~/apps/arduino
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.6_Linux_64bit.AppImage -O arduino-ide.AppImage
chmod +x arduino-ide.AppImage
./arduino-ide.AppImage --no-sandbox
```

---

### 2. Install Teensyduino

Follow PJRC’s official instructions:

👉 [https://www.pjrc.com/teensy/td_download.html](https://www.pjrc.com/teensy/td_download.html)

In Arduino IDE:

1. Open **Preferences**
2. Add the following URL to **Additional Boards Manager URLs**:

```
https://www.pjrc.com/teensy/package_teensy_index.json
```

3. Open **Tools → Board → Boards Manager**
4. Search for and install **Teensy**
5. Select **Teensy 4.1** as the active board

---

### 3. Install Required Arduino Libraries

From **Tools → Manage Libraries**, install:

* **Bounce2**

---

### 4. Install PJRC udev Rules

Required so Linux can access the Teensy without sudo:

```bash
cd ~/Downloads
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 5. Flash the Firmware

Firmware is located in:

```
annin_ar4_firmware/
```

#### Teensy 4.1

* Open the Teensy firmware sketch in Arduino IDE
* Select **Board: Teensy 4.1**
* Select the correct USB port
* Click **Upload**

#### Arduino Nano (Gripper)

* Open the gripper firmware sketch
* Select **Board: Arduino Nano**
* Select **Processor: ATmega328P (Old Bootloader)** if required
* Click **Upload**

---

### 6. Calibration Required

⚠️ After flashing firmware or power cycling the robot:

* A **calibration run is required**
* This is normal and expected behavior

Calibration is triggered automatically when launching the driver with:

```bash
calibrate:=True
```

---

At this point, the system is ready to be launched with real hardware.


### [Optional] Running in Docker Container

A docker container and run script has been provided that can be used to run the
robot and any GUI programs. It requires [rocker](https://github.com/osrf/rocker) to be installed. Then you can start the docker container with:

```bash
docker build -t ar4_ros_driver .

# Adjust the volume mounting and devices based on your project and hardware
rocker --ssh --x11 \
  --devices /dev/ttyUSB0 /dev/ttyACM0 \
  --volume $(pwd):/ar4_ws/src/ar4_ros_driver -- \
  ar4_ros_driver bash
```

## Usage

There are two modules that you will always need to run:

1. **Arm module** - this can be for either a real-world or simulated arm

   - For controlling the real-world arm, you will need to run the `annin_ar4_driver` module
   - For the simulated arm, you will need to run the `annin_ar4_gazebo` module
   - Either of the modules will load the necessary hardware descriptions for MoveIt

2. **MoveIt module** - the `annin_ar4_moveit_config` module provides the MoveIt interface and RViz GUI.

The various use cases of the modules and instructions to run them are described below:

---

### MoveIt Demo in RViz

If you are unfamiliar with MoveIt, it is recommended to start with this to explore planning with MoveIt in RViz. This contains neither a real-world nor a simulated arm but just a model loaded within RViz for visualisation.

The robot description, moveit interface and RViz will all be loaded in the single demo launch file

```bash
ros2 launch annin_ar4_moveit_config demo.launch.py ar_model:=mk4
```

---

### Control real-world arm with MoveIt in RViz

Start the `annin_ar4_driver` module, which will load configs and the robot description:

```bash
ros2 launch annin_ar4_driver driver.launch.py ar_model:=mk4 calibrate:=True include_gripper:=True
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `mk1`, `mk2`, `mk3` or `mk4`. Defaults to `mk4`.
- `calibrate`: Whether to calibrate the robot arm (determine the absolute position
  of each joint).
- `include_gripper`: Whether to include the servo gripper. Defaults to: `include_gripper:=True`.
- `serial_port`: Serial port of the Teensy board. Defaults to: `serial_port:=/dev/ttyACM0`.
- `arduino_serial_port`: Serial port of the Arduino Nano board. Defaults to `arduino_serial_port:=/dev/ttyUSB0`.

⚠️📏 Note: Calibration is required after flashing firmware to the Teensy board, and
power cycling the robot and/or the Teensy board. It can be skipped in subsequent
runs with `calibrate:=False`.

Start MoveIt and RViz:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

Available Launch Arguments:

- `ar_model`: The model of the AR4. Options are `mk1`, `mk2`, or `mk3`. Defaults to `mk3`.
- `include_gripper`: Whether to include the servo gripper. Defaults to:
  `include_gripper:=True`.
- `use_sim_time`: Make Moveit use simulation time. Should only be enabled when
  running with Gazebo. Defaults to: `use_sim_time:=False`.

You can now plan in RViz and control the real-world arm. Joint commands and joint states will be updated through the hardware interface.

NOTE: At any point you may interrupt the robot movement by pressing the E-Stop button
on the robot. This would abruptly stop the robot motion! To reset the E-Stop state of
the robot use the following command

```bash
ros2 run annin_ar4_driver reset_estop.sh <AR_MODEL>
```

where `<AR_MODEL>` is the model of the AR4, one of `mk1`, `mk2`, or `mk3`

---

### Control simulated arm in Gazebo with MoveIt in RViz

Start the `annin_ar4_gazebo` module, which will start the Gazebo simulator and load the robot description.

```bash
ros2 launch annin_ar4_gazebo gazebo.launch.py
```

Start Moveit and RViz:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true include_gripper:=True
```

You can now plan in RViz and control the simulated arm.

---

### Manual & Terminal Calibration Commands

The following ROS 2 commands are useful during setup, testing, and manual calibration.  
All commands assume your ROS workspace has been sourced.

#### Close Gripper

    ros2 action send_goal /gripper_controller/gripper_cmd \
      control_msgs/action/GripperCommand \
      "{command: {position: 0.012, max_effort: 0.0}}"

#### Open Gripper

    ros2 action send_goal /gripper_controller/gripper_cmd \
      control_msgs/action/GripperCommand \
      "{command: {position: 0.000, max_effort: 0.0}}"

#### Command Robot to Vertical Rest / Park Position

This is the **recommended pose when powering off the robot**.

    ros2 service call /park std_srvs/srv/Trigger "{}"

#### Manually Calibrate Selected Joints

    ros2 service call /calibrate_mask annin_ar4_driver/srv/CalibrateMask "{mask: '000011'}"

##### Calibration Mask Explanation

The calibration mask is a **6-character string**, one character per joint, ordered as:

    [J1][J2][J3][J4][J5][J6]

Each character may be:
- `1` → Calibrate this joint
- `0` → Skip this joint

Examples:
- `000011` → Calibrate **J5 and J6 only**
- `111111` → Calibrate **all joints**
- `100000` → Calibrate **J1 only**
- `001100` → Calibrate **J3 and J4 only**

This allows selective recalibration when only certain joints have been mechanically adjusted.

---

### Tuning Joint Offsets (Firmware-Level)

If your robot joints appear slightly misaligned after calibration (for example, a joint that is not perfectly vertical or horizontal when commanded to zero), joint offsets should be adjusted **directly in the Teensy firmware**, not in ROS configuration files.

Near the top of the Teensy sketch file, locate the following array:

    float CAL_OFFSET_DEG[NUM_JOINTS] = { 6.2, -3.8, 0, 0, 0, 0 };

This array defines a **per-joint angular offset in degrees** that is applied after calibration to compensate for small mechanical and assembly tolerances.

Joint index mapping:
- Index 0 → J1
- Index 1 → J2
- Index 2 → J3
- Index 3 → J4
- Index 4 → J5
- Index 5 → J6

#### How to Tune Joint Offsets

1. Perform a normal calibration sequence.
2. Command the robot to the vertical rest / park position.
3. Using a **digital level or angle gauge**, measure each joint.
4. If a joint is not aligned as expected:
   - Add a **positive value** if the joint must rotate further in the positive direction.
   - Add a **negative value** if the joint must rotate back in the negative direction.
5. Update the corresponding value in `CAL_OFFSET_DEG`.
6. Reflash the Teensy firmware and re-run calibration.

#### Example

If Joint 1 requires a **+6.2°** correction and Joint 2 requires a **−3.8°** correction:

    float CAL_OFFSET_DEG[NUM_JOINTS] = { 6.2, -3.8, 0, 0, 0, 0 };

Notes:
- Any change to `CAL_OFFSET_DEG` **requires reflashing the Teensy**
- These offsets are intended for **fine-tuning only**
- Large errors usually indicate a mechanical alignment issue

---

### Switching to Position Control

By default this repo uses velocity-based joint trajectory control. It allows the arm to move a lot faster and the arm movement is also a lot smoother. If for any
reason you'd like to use the simpler classic position-only control mode, you can
set `velocity_control_enabled: false` in [driver.yaml](./annin_ar4_driver/config/driver.yaml). Note that you'll need to reduce velocity and acceleration scaling in order for larger motions to succeed.

### Gripper Overcurrent Protection

See the [Gripper Overcurrent Protection](./docs/gripper_overcurrent_protection.md) page.
