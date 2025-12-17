# Indy7 Simulation & Gripper Control Guide

This document provides step-by-step instructions to launch the Indy7 simulation and control the gripper using ROS 2 CLI.

## 🚀 Step 1: Launch Simulation
**[Terminal 1]**
Open a terminal and run the main launch file. This starts Gazebo, the robot model, and the controllers.

```bash
ros2 launch indy_openvla indy_openvla_gazebo_base.launch.py
```

> **Note:** Wait until you see the log message `[joint_trajectory_controller]: Active` before proceeding.

---

## 🎮 Method A: Partial Joint Control (Recommended)
**Use this method to control ONLY the fingers.**

### 1. Configure Controller
**[Terminal 2]** Run this command **ONCE** to allow controlling fingers without arm values.
```bash
ros2 param set /joint_trajectory_controller allow_partial_joints_goal true
```
*Expected Output:* `Set parameter successful`

### 2. Gripper Commands (Fingers Only)
> **ℹ️ Tip:** We use `sec: 3` to move slowly and avoid physics jamming.

**✅ Open Gripper (Wide)**
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['left_finger_joint', 'right_finger_joint'],
  points: [{
    positions: [0.05, 0.05],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"
```

**✅ Close Gripper**
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['left_finger_joint', 'right_finger_joint'],
  points: [{
    positions: [0.0, 0.0],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"
```

---

## 🦾 Method B: Full Body Control (All Joints)
**Use this method if you cannot set parameters or prefer to send full robot states.**
*These commands hold the arm at the 'Home' position (`0, 0, -1.57, 0, -1.57, 0`) while moving the fingers.*

**✅ Open Gripper (with Arm Hold)**
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
    'left_finger_joint', 'right_finger_joint'
  ],
  points: [{
    positions: [0.0, 0.0, -1.57, 0.0, -1.57, 0.0, 0.05, 0.05],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"
```

**✅ Close Gripper (with Arm Hold)**
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
    'left_finger_joint', 'right_finger_joint'
  ],
  points: [{
    positions: [0.0, 0.0, -1.57, 0.0, -1.57, 0.0, 0.0, 0.0],
    time_from_start: {sec: 3, nanosec: 0}
  }]
}"
```

