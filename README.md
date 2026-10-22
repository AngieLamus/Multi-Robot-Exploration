# fronteras_ros2_multirobot
[![Demostración de Anti-Vicsek en Gazebo](https://img.youtube.com/vi/s_eMQDOLYiU/0.jpg)](https://www.youtube.com/watch?v=s_eMQDOLYiU)

ROS 2 (Humble) package implementing a decentralized coordination architecture for multi-agent SLAM in unstructured indoor environments.

This repository provides a scalable solution for deploying multiple TurtleBot3 robots using the Navigation2 (Nav2) stack and SLAM Toolbox. Unlike standard launch configurations, this package integrates autonomous frontier-based exploration algorithms, territorial partitioning via Voronoi diagrams, and tactical Anti-Vicsek repulsion. The objective is to optimize map coverage and minimize trajectory overlap among autonomous agents without relying on a centralized planner.

## System Architecture
The system is composed of several core modules:
* **Decentralized Exploration:** Implemented in the active exploration node (`exploracion_multi`), responsible for independent frontier detection and goal allocation for each agent.
* **Tactical Coordination (Anti-Vicsek):** The `coordinacion_multi.py` node executes a hierarchical repulsion logic to prevent collisions and inefficient clustering (e.g., granting priority to `tb1` over `tb2` during close encounters).
* **Nav2 / SLAM Integration:** Full integration with `slam_toolbox` and `nav2_bringup`, adapted via namespaces for multiple instances (`tb1`, `tb2`).

## Prerequisites
* Ubuntu 22.04 LTS
* ROS 2 Humble
* TurtleBot3 packages (`turtlebot3_gazebo`, `turtlebot3_description`)
* Navigation packages (`nav2_bringup`, `slam_toolbox`, `robot_state_publisher`)
* Python dependencies (`numpy`, `scipy` for Voronoi geometric calculations)

## Installation
1. Clone this repository into the `src` directory of your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone [https://github.com/your_username/fronteras_ros2_multiagente.git](https://github.com/your_username/fronteras_ros2_multiagente.git)
