# Semantic Fetch Robot (ROS 2)

## Project Overview

The **Semantic Fetch Robot** project aims to develop a ROS 2–based robotic system capable of receiving high-level object requests and navigating within a simulated environment to retrieve them. The long-term goal is to allow a robot to interpret semantic commands such as *“fetch the bottle”* and perform the required navigation and interaction tasks autonomously.

This repository currently contains the **initial package setup and a minimal ROS 2 node**, developed as part of **Milestone 1**.

---

## Milestone 1 Objectives

Milestone 1 focuses on establishing the foundational ROS 2 package structure and implementing a basic node prototype.

The main objectives are:

* Create a ROS 2 Python package.
* Configure package dependencies and build files.
* Implement a minimal ROS 2 node.
* Ensure the package builds successfully within a ROS 2 workspace.

---

## Package Structure

The package follows the standard ROS 2 Python package layout.

```
semantic_fetch_robot
├── package.xml
├── resource
│ └── semantic_fetch_robot
├── semantic_fetch_robot
│ ├── __init__.py
│ └── semantic_fetch_node.py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### Key Files

**package.xml**
Defines package metadata and ROS dependencies.

**setup.py / setup.cfg**
Configure the Python build system and register executable nodes.

**semantic_fetch_node.py**
Contains a minimal ROS 2 node that demonstrates node creation and message publishing.

**test/**
Contains default ROS 2 testing scripts generated for Python packages.

---

## Implemented Node

### `semantic_fetch_node`

This node represents the initial prototype for the semantic fetch system.

Current functionality:

* Initializes a ROS 2 node.
* Publishes a simple status message.
* Logs messages to the ROS console.

This node serves as a **placeholder for future system components**.

---

## Planned System Architecture

Future milestones will expand the project into multiple interacting ROS nodes:

```
User Request
     │
     ▼
Object Request Node
     │
     ▼
Semantic Mapping Node
     │
     ▼
Navigation Controller
     │
     ▼
Robot Motion / Fetch Action
```

These modules will enable the robot to interpret semantic commands and navigate autonomously in a simulated environment.

---

## Dependencies

The package currently depends on the following ROS 2 packages:

* `rclpy`
* `std_msgs`
* `geometry_msgs`
* `nav_msgs`
* `sensor_msgs`

These dependencies support communication, robot motion commands, and sensor data handling.

---

## Future Work

Planned features for upcoming milestones include:

* Object request handling
* Semantic mapping of environment objects
* Robot navigation control
* Integration with a simulated robot environment
* Task execution for fetching requested objects

---

## Author

Eva

---

## License

Apache License 2.0