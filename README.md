# Learning ROS 2 Underwater Robot

A personal learning project for studying **ROS 2**, **Gazebo Sim**, underwater vehicle dynamics, and feedback control by building a simulated Autonomous Underwater Vehicle (AUV).

The main goal of this project was not to create a production-ready underwater robot, but to understand how a complete robotic system can be structured from the simulation layer up to manual and autonomous control.

The project currently includes a simulated AUV with buoyancy, hydrodynamic drag, thrusters, control surfaces, closed-loop controllers, and gamepad control.

---

## Project Status

The main learning objectives of this project have been completed.

The AUV can currently operate in two control modes:

### ROV Mode

Manual control using a gamepad:

- Forward / backward movement
- Pitch control
- Yaw-rate control
- Real-time switching between ROV and AUV modes

### AUV Mode

Higher-level autonomous control:

- Target depth control
- Target heading control
- Target forward speed control
- Automatic pitch stabilization
- Automatic yaw-rate stabilization

The control architecture is based on cascaded feedback controllers.

```text
Depth Target
    ↓
Depth Controller
    ↓
Desired Pitch
    ↓
Pitch PID
    ↓
Elevator Fins


Heading Target
    ↓
Heading Controller
    ↓
Desired Yaw Rate
    ↓
Yaw PI
    ↓
Differential Thruster Control


Speed Target
    ↓
Speed PI
    ↓
Forward Thrust
```

---

## Features

- ROS 2 based AUV simulation
- Gazebo Sim underwater environment
- Buoyancy simulation
- Hydrodynamic drag
- Two rear thrusters
- Two controllable elevator fins
- Fixed stabilizing fins
- IMU feedback
- Odometry feedback
- Pitch PID controller
- Depth controller
- Yaw-rate PI controller
- Heading controller
- Forward speed controller
- ROV / AUV control modes
- Gamepad control
- Gamepad disconnect failsafe
- Gazebo third-person follow camera support

---

## Software Used

This project was developed using:

- **ROS 2 Lyrical Luth**
- **Gazebo Sim Jetty**
- **Python 3**
- `ros_gz`
- `ros_gz_bridge`
- `ros_gz_sim`
- `ros2_control`
- `controller_manager`
- `robot_state_publisher`
- `joy`

The AUV model is defined using URDF together with Gazebo-specific simulation plugins.

---

## AUV Configuration

The simulated vehicle uses:

- 2 rear propellers for surge and yaw
- 2 rear elevator fins for pitch control
- Passive fixed stabilizers
- IMU sensor
- Odometry feedback
- Approximately **11.8 kg** simulated vehicle mass
- Approximately **1.38 m** total vehicle length

The main hull is approximately:

```text
Length: 1.046 m
Outer diameter: 0.110 m
Inner diameter: 0.100 m
```

The thrusters are limited in the controller to approximately:

```text
15 N per thruster
```

The simulation also includes approximate hydrodynamic damping coefficients for surge, sway, heave, roll, pitch, and yaw.

> The hydrodynamic parameters in this project are intended for simulation and learning purposes. They have not yet been fully validated using experimental or CFD data.

---

## Control Modes

### ROV Mode

The gamepad directly commands vehicle motion while the low-level controllers stabilize the AUV.

Current mapping:

```text
Left Stick Y  → Forward / Backward
Left Stick X  → Yaw Rate
Right Stick Y → Pitch
A Button      → Switch ROV / AUV Mode
```

The joystick does not directly control the actuators.

Instead:

```text
Joystick
    ↓
Desired Pitch / Yaw Rate / Throttle
    ↓
Low-Level Controllers
    ↓
Thrusters + Fins
```

---

### AUV Mode

In AUV mode, the operator commands higher-level targets instead.

```text
Left Stick Y  → Target Speed
Left Stick X  → Change Target Heading
Right Stick Y → Change Target Depth
A Button      → Switch back to ROV Mode
```

When the stick is released:

- depth target is maintained
- heading target is maintained
- speed target follows the commanded value

---

## Repository Structure

```text
auv_sim/
│
├── auv_sim/
│   ├── thruster_controller.py
│   ├── gamepad_controller.py
│   └── ...
│
├── config/
│   └── controllers.yaml
│
├── launch/
│   └── *.launch.py
│
├── meshes/
│   └── AUV mesh files
│
├── urdf/
│   └── auv.urdf
│
├── worlds/
│   └── auv_world.sdf
│
├── resource/
│
├── package.xml
├── setup.py
└── README.md
```

---

## Installation

Create a ROS 2 workspace:

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
```

Clone this repository:

```bash
git clone https://github.com/Ghalih-thelad/Learning-ROS-2-UnderwaterRobot.git
```

Return to the workspace:

```bash
cd ~/auv_ws
```

Install dependencies if necessary:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --packages-select auv_sim
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Running the Simulation

Connect a compatible game controller before launching if you want to use manual control.

Then run:

```bash
ros2 launch auv_sim <launch_file>.launch.py
```

Replace `<launch_file>` with the launch file included in the `launch/` directory.

The launch system starts:

```text
Gazebo Sim
    ↓
AUV model
    ↓
ros2_control
    ↓
Fin controllers
    ↓
Thruster controller
    ↓
joy_node
    ↓
Gamepad controller
```

You can verify the running nodes using:

```bash
ros2 node list
```

Typical nodes include:

```text
/joy_node
/gamepad_controller
/thruster_controller
/robot_state_publisher
/controller_manager
```

---

## Useful ROS 2 Topics

Some important topics used by the project are:

```text
/joy
/imu
/auv/odom

/auv/cmd_vel
/auv/control_mode

/auv/target_depth
/auv/target_heading
/auv/target_speed

/left_fin_controller/commands
/right_fin_controller/commands
```

---

## Manual Testing Without a Gamepad

ROV / AUV mode can also be changed manually.

ROV mode:

```bash
ros2 topic pub --once /auv/control_mode std_msgs/msg/String \
"{data: 'ROV'}"
```

AUV mode:

```bash
ros2 topic pub --once /auv/control_mode std_msgs/msg/String \
"{data: 'AUV'}"
```

Example target depth:

```bash
ros2 topic pub --once /auv/target_depth std_msgs/msg/Float64 \
"{data: 2.0}"
```

Example target speed:

```bash
ros2 topic pub --once /auv/target_speed std_msgs/msg/Float64 \
"{data: 0.5}"
```

Example 90-degree heading target:

```bash
ros2 topic pub --once /auv/target_heading std_msgs/msg/Float64 \
"{data: 1.5708}"
```

Heading values are internally represented in radians.

---

## What I Learned

This project was created mainly as a practical way to learn how different robotics concepts connect together.

Some of the topics explored include:

- ROS 2 nodes, topics, publishers, and subscribers
- URDF robot modeling
- Gazebo Sim plugins
- ROS 2 and Gazebo bridging
- `ros2_control`
- PID / PI feedback control
- Cascaded controllers
- Vehicle dynamics
- Hydrodynamic damping
- Buoyancy
- Thruster mixing
- Gamepad teleoperation
- Autonomous depth, heading, and speed control

One of the most useful lessons from this project was learning to separate the system into layers:

```text
User / Mission Commands
        ↓
High-Level Controllers
        ↓
Low-Level Controllers
        ↓
Actuators
        ↓
Vehicle Dynamics
        ↓
Sensors
        └──────────── feedback
```

This makes it possible to use the same low-level AUV controller with different input systems such as a gamepad, autonomous mission planner, or future navigation system.

---

## Future Work

Possible future additions include:

- Waypoint navigation
- Mission planning
- Autonomous path following
- Camera sensor
- Computer vision
- Sonar simulation
- Improved hydrodynamic coefficients
- Added mass modeling
- CFD-based parameter estimation
- Real AUV hardware integration
- Navigation using IMU + depth sensor
- SLAM / localization experiments

---

## Purpose

This repository documents my progress while learning ROS 2 and underwater robotics.

It is intentionally kept as a learning project so other students who are beginning with ROS 2, Gazebo Sim, AUVs, or robotic control systems can inspect the implementation, experiment with it, and build on top of it.

Contributions, suggestions, and improvements are welcome.

---

## Author

**Ghalih Ahmad Salamun**
Universitas Airlangga

GitHub: [Ghalih-thelad](https://github.com/Ghalih-thelad)
