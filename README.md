# Autonomous UR5 Pick & Place via MoveIt 2 and BehaviorTree.CPP

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?logo=gazebo)
![MoveIt 2](https://img.shields.io/badge/MoveIt-2-00599C)

An industrial-grade autonomous robotic pick-and-place sequence developed on **ROS 2 Jazzy** utilizing **MoveIt 2** for kinematic trajectory planning and **BehaviorTree.CPP v4** for high-level control-flow orchestration. The physical environment and robot dynamics are fully simulated inside **Gazebo Harmonic**.

##  System Architecture

This project strictly separates the decision-making logic ("The Brain") from the kinematic execution and hardware interfaces ("The Body") using Object-Oriented Programming (OOP) principles.

* **Robot Manipulation:** Universal Robots UR5 (6-DOF)
* **End Effector:** Robotiq 2F-85 Parallel Gripper
* **Decision Framework:** BehaviorTree.CPP 
* **Trajectory Generation:** OMPL (RRTConnect) parameterized via Time-Optimal Trajectory Generation (TOTG)
* **Physics Engine:** Gazebo Harmonic via `gz_ros2_control`

### Core Features
* **Modular BT.CPP Architecture:** Custom, decoupled C++ action nodes (`MoveArmAction`, `ControlGripperAction`) act as wrappers around ROS 2 action servers and the MoveIt API.
* **Persistent Joint Constraints:** The `MoveGroupInterface` is initialized globally to sustain system joint states via active `/joint_states` broadcasts, preventing "phantom state" planning failures.
* **Synchronized Simulation Clocks:** Injected `use_sim_time` parameters to prevent timestamp drift issues inside the OMPL pipeline.
* **Mock Hardware Interfaces:** Utilizes ROS 2 `mock_components` for the Robotiq gripper to bypass Gazebo Classic plugin incompatibilities in Harmonic, ensuring MoveIt receives a valid, continuous joint state stream.

---

##  Prerequisites

Ensure you have the following installed on an Ubuntu 24.04 (Noble) system:
* [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
* [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)
* MoveIt 2 for ROS 2 Jazzy
* BehaviorTree.CPP (v4)
* `ros-jazzy-control-msgs`

---

