---
layout: default
title: "Milestone 1: Proposal & Architecture"
---

# Milestone 1: Proposal & Architecture

<div style="display:flex;gap:1rem;flex-wrap:wrap;margin-bottom:2rem;">
  <span class="badge badge-success">✓ Submitted</span>
  <span class="badge badge-info">Due: Mar 6, 2026</span>
  <span class="badge badge-info">Weight: 5%</span>
  <span class="badge badge-info">~1,500 words</span>
</div>

---

## 1. Mission Statement & Scope

The **Semantic Fetch Robot** is a type of mobile manipulation system, which connects the natural language commands of humans with autonomous robotic retrieval of items. The robot operates in a structured indoor simulation and has one main task: to go to the item indicated by the operator (i.e., when the operator says *"fetch the red bottle"*) and retrieve it by picking it up using an arm attached to the robot. After the robot has picked up the item it will travel back to the operator's location with the item.

**Environment:** The simulation takes place in a pre-generated world for a TurtleBot that has all the required features (shelves, walls, and corridors) to create a realistic warehouse environment. We will use Gazebo SDF model-simulation feature which allows us to place objects in a specific location within the simulation world.

**Primary problem being solved:** Most indoor robot systems are designed to either navigate or manipulate, but the Semantic Fetch Robot will coordinate the two actions based upon the semantic command from the operator. Achieving this capability requires a tight integration of distinct capabilities in the software and hardware used by the robot to accomplish the task. The project develops and verifies that the complete system is functioning correctly in the simulation.

**Success state:** ≥ 75% object fetch success rate over 10 trials, no collisions, task completed within 180 seconds per run, in a pre-mapped environment.

---

## 2. Background & Prior Work

Our project sits at the intersection of three active research areas: semantic mapping, open-vocabulary object detection, and mobile manipulation. Below is the prior work that shaped our approach.

**Mobile Manipulation / Fetch Robots.** The RoboCup@Home competition has driven development of fetch-capable service robots for over a decade. Common architectures follow a pipeline of map → localize → detect → plan → grasp, which we adopt. A key insight from this literature is that decoupling navigation from manipulation using separate planners communicating via a coordination layer which is more robust than tightly coupled systems.

**Open-Vocabulary Object Detection.** When using traditional YOLO models, the model relies on an unchanging list of classes, preventing its utility if a user asks for any number of items at once. There has been an increase in work geared towards open-ended or "open-vocabulary" models. **CLIP** (Radford et al., OpenAI, 2021) demonstrated that aligning visual and textual features allows for zero-shot recognition of new classes through the use of visual embeddings and textual embeddings. This functionality is integrated into **YOLOWorld** (Cheng et al., 2024) by directly adding CLIP-type text encoders to the YOLO detection architecture providing the ability to perform open-everything in real time, thus addressing our limitations for embedded compute. The **OpenNav** framework (arxiv:2408.13936) built a complete ROS 2 pipeline combining YOLOWorld + MobileSAM to perform open-vocabulary 3D object detection, which was integrated directly into a navigation stack and exhibited state-of-the-art mAP with respect to the Replica dataset.

**Semantic Mapping.** ConceptGraphs (Gu et al., 2023) and DualMap (2025) both show how to take object detections in 2D images and translate those into 3D space and create a map that can be queried against. This is the technique we will utilize. Specifically, DualMap demonstrates this component in a ROS 2 pipeline using a wheeled robot, LiDAR and RGB-D camera, which is the same basic hardware setup as we will use.

**SLAM & Navigation.** For 2D mapping, SLAM Toolbox is the official ROS 2 SLAM package. For the navigation of TurtleBot navigation, Nav2 provides an action server for navigation, layers of costmaps, and recovery behaviors. Both packages have been tested against the TurtleBot simulator and are supported on both platforms.

**MoveIt 2 for Arm Control.** MoveIt 2 is the standard ROS 2 motion planning framework. The `open_manipulator_x_moveit_config` package (ROBOTIS) provides pre-built MoveIt 2 configuration for the OpenMANIPULATOR-X, including URDF, SRDF, joint limits, and IK solver setup.

---

## 3. Technical Specifications

| Parameter | Value |
|---|---|
| **Robot Platform** | TurtleBot |
| **Mounted Arm** | OpenMANIPULATOR-X (4-DOF + gripper, DYNAMIXEL XM430) |
| **Kinematic Model — Base** | Differential Drive (iRobot Create 3) |
| **Kinematic Model — Arm** | Serial chain, 4 revolute joints + parallel gripper |
| **Sensors** | OAK-D Spatial AI stereo camera, RPLIDAR A1 2D LiDAR, IMU |
| **Simulator** | Gazebo Harmonic (`gz-harmonic`) |
| **Simulation World** | `depot.sdf` — TurtleBot default depot world |
| **ROS Version** | ROS 2 Jazzy Jalisco |
| **OS** | Ubuntu 24.04 LTS |

---

## 4. Simulation Environment

We are using **Gazebo Harmonic** (`gz-harmonic`), which is the officially supported simulator for ROS 2 Jazzy and the TurtleBot platform. The TurtleBot simulator package (`ros-jazzy-turtlebot4-simulator`) ships with built-in support for Gazebo Harmonic and provides an out-of-the-box simulation launch:

```bash
sudo apt install gz-harmonic ros-jazzy-turtlebot4-simulator
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  slam:=true nav2:=true rviz:=true
```

**Chosen World:** `depot.sdf` — the TurtleBot default depot warehouse world. This environment was chosen because:
- It ships pre-built with `turtlebot4-simulator`, requiring no custom world authoring
- A pre-built map (`depot.yaml`) is available, enabling Nav2 localization from day one
- The warehouse layout (corridors, open shelving areas) is representative of real service robot environments
- It is large enough to make navigation non-trivial but small enough to run on a single development machine

**Object Placement.** Target objects (bottles, cans, small boxes from the Gazebo Fuel model database) will be spawned at fixed, known locations in the world using SDF `<include>` tags. This eliminates random placement variability in early milestones. In Milestone 3, we will introduce varied placement to test robustness.

---

## 5. Robot Arm Integration

**Platform compatibility note.** The OpenMANIPULATOR-X is natively designed for TurtleBot3 (Waffle/Waffle Pi). TurtleBot uses a different base (iRobot Create 3) and does not have an official combined URDF for TB4 + OpenMANIPULATOR-X. Our approach to resolving this is described below.

**URDF Integration Strategy.** We will compose a combined robot description by extending the TurtleBot URDF with a `<xacro:include>` for the OpenMANIPULATOR-X description from the `open_manipulator_x_description` package. The arm will be attached to the TurtleBot  top mounting plate via a fixed joint, with the URDF offset tuned to match the physical mounting position. This approach is consistent with how the TurtleBot3 manipulation packages compose their combined description.

**Key packages used:**

| Package | Source | Purpose |
|---|---|---|
| `turtlebot4_description` | `ros-jazzy-turtlebot4` | TurtleBot base URDF/xacro |
| `open_manipulator_x_description` | ROBOTIS GitHub (jazzy branch) | Arm URDF and mesh files |
| `open_manipulator_x_moveit_config` | ROBOTIS GitHub | Pre-built MoveIt 2 config, IK solver, SRDF |
| `ros2_control` | apt | Hardware abstraction layer for arm joints |
| `moveit2` | `ros-jazzy-moveit` | Motion planning framework |

**Simulation arm control.** In Gazebo Harmonic, the arm joints are controlled via the `ros2_control` Gazebo plugin using the `JointTrajectoryController`. The MoveIt 2 `moveit_gazebo.launch.py` from `open_manipulator_x_moveit_config` launches the planning context and bridges it to the sim controllers. Grasp execution uses MoveIt's `MoveGroupInterface` to plan and execute a pre-computed approach trajectory.

**Grasp pose estimation.** The OAK-D camera provides both RGB and registered depth. After the object detector produces a 2D bounding box, we project the centroid pixel through the depth map to obtain a 3D point in the camera frame. This 3D point is transformed to the `base_link` frame using `tf2`, then used as the target for a top-down grasp pose. MoveIt 2 solves IK for the approach and grasp configurations.

---

## 6. Open-Source Stack & Build vs. Reuse Decisions

A core principle of this project is to **reuse well-maintained open-source packages wherever possible** and write custom code only where no adequate solution exists. The table below documents every major software component and the decision made.

| Capability | Chosen Open-Source Package | Decision | Rationale |
|---|---|---|---|
| SLAM / Mapping | `slam_toolbox` (ROS 2 Jazzy) | **Reuse** | Ships with TurtleBot |
| EKF Localization | `robot_localization` | **Reuse** | Industry standard for sensor fusion on mobile robots |
| Navigation | `nav2` | **Reuse** | Standard ROS 2 navigation stack |
| Object Detection | YOLOWorld (`ultralytics`) | **Reuse (wrap)** | Open-vocabulary, real-time, Python API available |
| Arm Motion Planning | `moveit2` + `open_manipulator_x_moveit_config` | **Reuse** | Full IK/planning config pre-built by ROBOTIS |
| Arm URDF | `open_manipulator_x_description` | **Reuse** | Official ROBOTIS description package |
| Depth Projection | `image_geometry` + `tf2` | **Reuse** | Standard ROS 2 perception utilities |
| Semantic Map | Custom `semantic_map_server` node | **Custom** | No standard ROS 2 package for queryable object registries |
| Command Parser | Custom `fetch_command_node` | **Custom** | Bridges text input → object label → Nav2 goal |
| Grasp Coordinator | Custom `grasp_coordinator_node` | **Custom** | Integrates detection pose → MoveIt 2 execution |

---

## 7. High-Level System Architecture

The system follows a **Perception → Estimation → Planning → Actuation** flow with two additional coordination layers: a **Semantic Layer** that maintains the object registry, and a **Task Layer** that sequences the full fetch behavior (navigate → detect → grasp → return).

**Control strategy summary.** The base uses Nav2's velocity smoother outputting to `/cmd_vel`, translated to wheel commands by the iRobot Create 3 firmware. The arm uses `ros2_control` JointTrajectoryController, commanded by MoveIt 2 via the `FollowJointTrajectory` action. The two controllers operate independently — the base is stopped before arm motion begins, preventing simultaneous base/arm movement that could destabilize the platform.

---

## 8. Package Structure

The ROS 2 package is organized as follows:

```
semantic_fetch_robot/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── launch/
│   └── bringup.launch.py
├── config/
│   ├── nav2_params.yaml
│   └── moveit_params.yaml         
├── semantic_fetch_robot/
│   ├── __init__.py
│   ├── semantic_fetch_node.py
└── test/
    └── test_node.py
```

### Milestone 1 Nodes

`semantic_fetch_node.py` confirms the ROS 2 package builds and runs correctly. It initializes the node and publishes a status heartbeat, serving as the foundation for all subsequent nodes.

<div style="margin-top:2rem;padding-top:1rem;border-top:1px solid var(--border);display:flex;justify-content:space-between;font-family:var(--mono);font-size:0.75rem;color:var(--text-dim);">
  <span>← <a href="/">Overview</a></span>
</div>
