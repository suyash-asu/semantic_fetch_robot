---
layout: default
title: Overview
has_toc: true
nav_order: 1
---

<div class="hero">
  <div class="hero-tag">RAS 598 - Mobile Robotics</div>
  <h1>Semantic Fetch Robot</h1>
  <p class="hero-desc">
    A ROS 2 mobile manipulation system that accepts a natural-language object request, navigates an indoor environment, locates the target using open-vocabulary vision, and physically retrieves it using a mounted robotic arm.
  </p>
  <div class="hero-meta">
    <span class="meta-chip">TurtleBot + OpenMANIPULATOR-X</span>
    <span class="meta-chip"> Gazebo Harmonic</span>
    <span class="meta-chip">Differential Drive</span>
    <span class="meta-chip">LiDAR + OAK-D RGB-D</span>
    <span class="badge badge-info" style="margin-left:auto;">Active Development</span>
  </div>
</div>

## On this page

- [Project Statement](#project-statement)
- [The Problem We Are Solving](#the-problem-we-are-solving)
- [Project Components](#project-components)
- [Technical Specifications](#technical-specifications)
- [Current Status](#current-status)

## Project Statement

The Semantic Fetch Robot is a ROS 2 mobile manipulator operating in a mapped indoor warehouse environment (TurtleBot default Gazebo Harmonic depot world). Given a text command such as "fetch the red bottle", it autonomously navigates to the relevant region, locates the requested object using open-vocabulary visual detection, grasps it with the mounted OpenMANIPULATOR-X arm, and returns to the operator to deliver the item.

**Success state:** The robot correctly identifies, grasps, and delivers the requested object in ≥ 75% of trials within a pre-mapped simulation environment, without collisions.

## The Problem We Are Solving

Most service robots either move well or manipulate well, but few can do both while also understanding flexible, natural-language requests. Semantic Fetch aims to bridge this gap by enabling a mobile robot to understand what object is being requested, infer where it is likely to be, navigate to it, and bring it back safely.

To achieve this, we combine SLAM-based (Simultaneous Localization and Mapping) navigation with semantically grounded object detection and arm motion planning in a single ROS 2 pipeline. The project focuses on building realistic simulations that serve as a proof of concept before deployment to real hardware.

## Project Components

<div class="card-grid">
  <div class="card">
    <h3>Semantic Mapping</h3>
    <p>SLAM-built occupancy grid enriched with object detections. Every item detected is registered in a queryable map with its 3D location and semantic label.</p>
  </div>
  <div class="card">
    <h3>Open-Vocabulary Detection</h3>
    <p>YOLOWorld or CLIP-based detection on the OAK-D camera stream which allows the robot to find objects described in free text without a fixed class list.</p>
  </div>
  <div class="card">
    <h3>Autonomous Navigation</h3>
    <p>Nav2 stack with SLAM Toolbox handles global path planning and obstacle avoidance. The robot navigates to object locations queried from the semantic map.</p>
  </div>
  <div class="card">
    <h3>Arm Control & Grasping</h3>
    <p>MoveIt 2 plans collision-free arm trajectories for the OpenMANIPULATOR-X. Grasp poses are computed from RGB-D point cloud data.</p>
  </div>
</div>

## Technical Specifications

| Parameter | Value |
|---|---|
| **Robot Platform** | TurtleBot Standard + OpenMANIPULATOR-X |
| **Kinematic Model** | Differential Drive (base) + Serial 4-DOF (arm) |
| **Primary Sensors** | OAK-D Spatial AI Stereo Camera, RPLIDAR A1 2D LiDAR, IMU |
| **Simulation Engine** | Gazebo Harmonic (gz-harmonic) |
| **Simulation World** | TurtleBot default depot world |
| **ROS Version** | ROS 2 Jazzy Jalisco |
| **OS** | Ubuntu 24.04 LTS |

## Current Status

- Milestone 1 (proposal and architecture) has been completed; details are documented in [Milestone 1](milestone1.md).
- Milestone 2 (Implementation, integration, and evaluation in simulation) is completed: details are documented in [Milestone 2](milestone2.md).
- Object spawning + full pick-and-place pipeline are planned next.