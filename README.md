# ros-embodied-uav-agent (Gazebo UAV Delivery Demo)

A ROS + Gazebo demo for an embodied UAV agent task:

**wait → takeoff → cruise → descend & hover → drop payload → climb → cruise → land**

This repo is for the course homework: **03-Agent-Embodied**.

## Demo
- Video: (to be updated)
- Code: https://github.com/CH-YXZ/ros-embodied-uav-agent

## Environment
- Ubuntu 20.04
- ROS Noetic
- Gazebo9 (`gazebo_ros`)

## Project Structure
- `ros_ws/src/uav_gazebo_demo/` : main ROS package
  - `launch/demo.launch` : start Gazebo + spawn UAV/payload + controllers
  - `models/iris_visual_only/` : visual-only UAV model (no PX4 plugins)
  - `models/payload_box/` : payload model
  - `scripts/` : control / drop / executor scripts
  - `results/traces/` : trace logs output (if enabled)

## Install (ROS Noetic)
If you already have ROS Noetic, you can skip.

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full ros-noetic-gazebo-ros-pkgs
