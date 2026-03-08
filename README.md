# Clankers Robot Navigation System

An autonomous ROS 2 robotics system designed to map an unknown Gazebo environment and execute a topological delivery mission. This project is split into two distinct operational phases: **Phase 1** (Active SLAM and Autonomous Frontier Exploration) and **Phase 2** (Static Map Navigation and Delivery via AMCL).

## Tech Stack
* **ROS 2 Jazzy**: Middleware and communication framework.
* **Gazebo**: Physics and 3D simulation environment.
* **Nav2 (Navigation 2)**: Core path-planning utilizing the `MPPIController` for dynamic trajectory generation.
* **SLAM Toolbox**: Synchronous mapping and loop closure.
* **AMCL**: Probabilistic localization against a static map.
* **Python 3, NumPy, SciPy**: Custom node development for matrix math and computer vision.

## Core Architecture
* **Frontier Exploration Node**: Converts the dynamic `/map` topic into a 2D NumPy array, dilates unknown space, and identifies boundaries. Ranks exploration targets using an **Information Gain** heuristic (`cost = distance - (blob_size * 0.05)`) to mathematically hunt down the largest unmapped areas.
* **Dynamic Watchdog**: A proximity-based fail-safe that aborts Nav2 goals once the robot's LiDAR physically "sees" the target area (Visual Range Victory), preventing the robot from freezing against mathematically unreachable walls.
* **Phase Separation**: Strictly separates mapping (SLAM) and navigating (AMCL) to prevent Transform (TF) tree conflicts and optimize compute power.

---

## Setup & Execution Guide

### Prerequisites & Workspace Setup
Ensure you are running **ROS 2 Jazzy** on Ubuntu and have the Nav2 and SLAM Toolbox packages installed. 

Before running any commands, build the workspace and source the overlay in every terminal you open:
```bash
cd ~/ros2_ws
colcon build --packages-select mission_planner my_robot_nav
source install/setup.bash
```

---

### Phase 1: Autonomous Mapping (Active SLAM)
To map a new environment from scratch, you will need 5 separate terminals.

**Terminal 1: The Physics Engine**
Launch your Gazebo simulation to spawn the robot and the world.
```bash
# (Execute your specific Gazebo launch file here)
ros2 launch my_robot_nav <gazebo_launch_file.py>
```

**Terminal 2: The Map Maker**
Launch the SLAM Toolbox to process LiDAR and odometry into a dynamic map.
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Terminal 3: The Navigator**
Launch Nav2 without a static map to enable dynamic path planning.
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/madkane/ros2_ws/src/clankers_robot_project/my_robot_nav/config/nav2_params.yaml
```

**Terminal 4: Visualization**
Open RViz to monitor the mapping progress.
```bash
rviz2 -d /home/madkane/ros2_ws/src/clankers_robot_project/my_robot_nav/rviz/mapping.rviz
```

**Terminal 5: The Autonomous Brain**
Run the custom Python script that analyzes the `/map` matrix and hunts down unexplored frontiers.
```bash
ros2 run mission_planner frontier_explorer
```

*(Optional) Manual Teleoperation Override:*
If the robot gets stuck or you want to manually finish the map, kill Terminal 3 and 5, then run:
```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

**Saving the Map:**
Once the floor plan is fully enclosed in RViz, save it permanently. Run this in a fresh terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/clankers_robot_project/my_robot_nav/maps/delivery_maze_map
```
*Note: Ensure the generated `.yaml` file contains the absolute path to the `.pgm` image.*

---

### Phase 2: The Delivery Mission (AMCL Navigation)
Once the static map is saved, SLAM is retired. You only need 4 terminals for the delivery phase.

**Terminal 1: The Physics Engine**
Launch your Gazebo simulation just like in Phase 1.
```bash
ros2 launch my_robot_nav <gazebo_launch_file.py>
```

**Terminal 2: Nav2, Map Server & AMCL**
Boot the entire navigation stack, providing the static map you generated in Phase 1.
```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/madkane/ros2_ws/src/clankers_robot_project/my_robot_nav/maps/delivery_maze_map.yaml params_file:=/home/madkane/ros2_ws/src/clankers_robot_project/my_robot_nav/config/nav2_params.yaml
```

**Terminal 3: Visualization & AMCL Wake-Up**
Open RViz. **Crucial Step:** You must use the "2D Pose Estimate" button in the RViz toolbar to click and drag the robot's initial starting location on the map. AMCL will not bridge the TF tree until you provide this initial guess.
```bash
rviz2 -d /home/madkane/ros2_ws/src/clankers_robot_project/my_robot_nav/rviz/navigation.rviz
```

**Terminal 4: The Delivery & Battery Logic**
Once the robot is localized in RViz, launch the battery simulator and the delivery mission planner. 
*Note: It is recommended to run these in two separate terminal tabs to avoid creating background "zombie" processes that survive a `Ctrl+C` command.*

```bash
# In Terminal 4A:
ros2 run mission_planner battery_simulator

# In Terminal 4B:
ros2 run mission_planner mission_planner
```