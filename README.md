# RoboticsLab_Technical_Project
Robotics Lab 2025/2026 - Technical Project - Cooperative Robots for NDI Inspection

---

## Overview

This project presents a **collaborative robotic inspection pipeline** developed in **ROS 2 Humble**.

The simulated scenario reproduces a simplified industrial **Non-Destructive Inspection (NDI)** workflow in which:

- a **KUKA iiwa** manipulator performs a **visual inspection** of a component using an onboard camera,
- a **fra2mo** mobile robot carries the inspected part and later executes the logistic task,
- the inspection result is classified as:
  - **ADMIT** if the part is conformant,
  - **REJECT** if the part is defective,
- according to the inspection outcome, the mobile robot autonomously navigates toward the appropriate deployment area:
  - **green zone** for conformant parts,
  - **red zone** for defective parts.

The objective is to integrate **manipulation**, **computer vision**, and **mobile navigation** inside a single ROS 2-based architecture.

---

## Scenario Description

The custom simulation world represents an industrial inspection area with:

- perimeter walls,
- narrow passages,
- obstacles,
- a work area for the inspection task,
- a **green deployment zone**,
- a **red deployment zone**.

The manipulator inspects the component visually, while the mobile robot executes the final transport task.

---

## Main Technologies

- **ROS 2 Humble**
- **Gazebo / ros_gz**
- **Nav2**
- **OpenCV**
- **cv_bridge**
- **KDL**
- **RViz2**
- **Python + C++ ROS nodes**

---

## Repository Structure

```text
TechProject/
├── iiwa_bringup/
├── iiwa_controllers/
│   ├── admittance_controller/
│   ├── external_torque_sensor_broadcaster/
│   └── impedance_controller/
├── iiwa_description/
├── iiwa_hardware/
├── ros2_fra2mo/
├── ros2_kdl_package/
├── simulation_launch/
└── m-explore-ros2/
```
---
### Main packages

-   **iiwa\_bringup**  
    Launch files for the iiwa robot.
    
-   **iiwa\_description**  
    Robot description, controllers configuration, meshes and Gazebo resources.
    
-   **iiwa\_controllers**  
    Custom controllers used by the iiwa stack.
    
-   **ros2\_kdl\_package**  
    KDL-based control/action server used for the inspection motion.
    
-   **ros2\_fra2mo**  
    Mobile robot stack, maps, navigation configuration and deployment logic.
    
-   **simulation\_launch**  
    Custom simulation package containing:
    -   the main simulation launch file,
    -   the custom world resources,
    -   custom models,
    -   RViz configurations,
    -   the computer vision inspection node.  

* * *

## Functional Workflow

The full workflow is the following:

1.  the simulation world is launched,
2.  both robots are spawned,
3.  the **iiwa** action server starts,
4.  the **inspection CV node** listens to the camera topic,
5.  the iiwa executes the inspection motion,
6.  the CV node detects whether the inspected component contains a red defect,
7.  the inspection result is published on `/inspection_result`,
8.  the **fra2mo** robot waits for the outcome and navigates to:
    -   the **green area** if the part is accepted, 
    -   the **red area** if the part is rejected. 

* * *

## Relevant Topics and Interfaces

### Inspection

-   `/iiwa/camera`
-   `/inspection_active`
-   `/inspection_result`
    
### Manipulator

-   `/kdl_feedback` (action server)
    
### Navigation

-   Nav2 stack through `nav2_simple_commander` 
-   goal execution handled by `reach_goal.py`
    
* * *

## Computer Vision Logic

The inspection is implemented in the node:
```
simulation_launch/scripts/inspection\_cv\_node.py
```
The node:
-   subscribes to the iiwa camera topic,
-   becomes active only when inspection is enabled,
-   converts ROS images to OpenCV images via `cv_bridge`,
-   detects the amount of **red area** in the image,
-   compares the detected area against a threshold,
-   publishes:
    -   `ADMIT` if the red area is below threshold, 
    -   `REJECT` if the red area exceeds threshold.
        
This simulates a simplified defect detection stage in an industrial quality-control pipeline.

* * *

## Prerequisites

Tested on:
-   **Ubuntu 22.04**  
-   **ROS 2 Humble** 

Install the main dependencies:
```bash
sudo apt update  
  
sudo apt install -y \  
  ros-humble-ros-gz-sim \  
  ros-humble-ros-gz-bridge \  
  ros-humble-ros-gz-interfaces \  
  ros-humble-ros2-control \  
  ros-humble-ros2-controllers \  
  ros-humble-controller-manager \  
  ros-humble-joint-state-broadcaster \  
  ros-humble-position-controllers \  
  ros-humble-velocity-controllers \  
  ros-humble-effort-controllers \  
  ros-humble-joint-trajectory-controller \  
  ros-humble-navigation2 \  
  ros-humble-nav2-bringup \  
  ros-humble-nav2-simple-commander \  
  ros-humble-slam-toolbox \  
  ros-humble-robot-localization \  
  ros-humble-cv-bridge \  
  ros-humble-image-transport \  
  ros-humble-xacro \  
  ros-humble-kdl-parser \  
  liborocos-kdl-dev \  
  ros-humble-rqt-image-view
```
If `rosdep` is not initialized yet:
```bash
sudo rosdep init  
rosdep update
```
* * *

## Workspace Setup

The project is meant to be placed under:
```
~/ros2_ws/src/TechProject
```
Example:
```bash
mkdir -p ~/ros2_ws/src  
cd ~/ros2_ws/src  
git clone <YOUR_REPOSITORY_URL> TechProject
```
Then build from the workspace root:
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
rosdep install --from-paths src --ignore-src -r -y  
colcon build --symlink-install  
source install/setup.bash
```
* * *

## Build Notes

The workspace must be built from:
```
~/ros2\_ws
```
not from the package directory.

If other old homework folders are present in `~/ros2_ws/src` and contain duplicated package names, remove them from the workspace or move them elsewhere before building.

* * *

## Running the Full Demo

The recommended execution is with **5 separate terminals**.

### Terminal 1 — Simulation
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
ros2 launch simulation_launch simulation.launch.py
```
### Terminal 2 — KDL / Manipulator control
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
ros2 launch ros2_kdl_package ros2_kdl.launch.py
```
### Terminal 3 — Computer vision inspection node
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
ros2 run simulation_launch inspection_cv_node.py
```
### Terminal 4 — Mobile robot navigation
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
### Terminal 5 — Final task execution
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
ros2 run ros2_fra2mo reach_goal.py
```
* * *

## Viewing the Camera Stream

To inspect what the camera sees:
```bash
source /opt/ros/humble/setup.bash  
cd ~/ros2_ws  
source install/setup.bash  
rqt_image_view
```
Then select the camera topic, typically: ` /iiwa/camera`
You can also inspect the available image topics with:
```bash
ros2 topic list | grep -E "camera|image"
```
* * *

## Important Files

### Main simulation launch
```
simulation_launch/launch/simulation.launch.py
```
### Navigation launch
```
ros2_fra2mo/launch/fra2mo_navigation.launch.py
```
### Inspection node
```
simulation_launch/scripts/inspection_cv_node.py
```
### Final coordination script
```
ros2_fra2mo/scripts/reach_goal.py
```
### KDL parameters
```
ros2_kdl_package/config/args.yaml
```
### Main world
```
ros2_fra2mo/worlds/leonardo_inspection_world.sdf
```
* * *

## Expected Result

At the end of the demo:
-   the manipulator performs the inspection motion,
-   the system classifies the part as conformant or defective,
-   the inspection result is published,
-   the mobile robot reaches the corresponding deployment zone.

This demonstrates the integration of:

-   **robot manipulation**,
-   **computer vision**,
-   **autonomous navigation**,
-   **ROS 2 system orchestration**

within a realistic industrial-inspired use case.
