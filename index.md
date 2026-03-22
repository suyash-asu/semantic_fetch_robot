---
layout: default
title: Overview
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

## Project Statement

The *Semantic Fetch Robot* works in a *mapped indoor environment* i.e. TurtleBot default Gazebo’s Harmonics depot / warehouse world. The human operator gives commands to fetch an object (e.g., red bottle) via text command. The robot will then do the following:

1. Navigate autonomously through the mapped environment.
2. Locate the target item with open vocabulary visual detection. 
3. Approach and pick up the item using the mounted OpenMANIPULATOR-X arm. 
4. Return to the operator’s starting point to deliver the item.

**Success state:** The robot correctly identifies, grasps, and delivers the requested object in ≥ 75% of trials within a pre-mapped simulation environment, without collisions.

## The Problem We Are Solving

There are very few robots that are good at moving and also be able to pick things up. It's also not very common for them to respond to verbal commands. The additional bridge between the two capabilities is *Semantic Fetch*.  In order for the robot to succeed, they must know exactly what to get, as well as the location of the object to be picked up, how to get to that object, and return to the starting point with the object. To accomplish this task, the components of SLAM-based (Simultaneous Localization and Mapping) navigation will be combined with the use of semantically grounded object (robotic) detection, and arm motion planning, all integrated into one pipeline running on ROS 2.

This is a well-known and studied area of service robotics, and as such the goal of the project is to develop simulations that will serve as the final proof of concept prior to implementation of the developed system to real hardware robots.

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