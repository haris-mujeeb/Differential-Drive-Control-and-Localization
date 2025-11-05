# Bumperbot ROS 2 Project

This README provides a step-by-step guide to setting up and running the Bumperbot robot simulation using ROS 2 and Gazebo.

## 1. Project Overview

The `bumperbot_description` package contains all the necessary files to describe the Bumperbot robot, including its URDF model, meshes, launch files for simulation, and Rviz configuration.

<img width="242" height="304" alt="image" src="https://github.com/user-attachments/assets/96656d5c-39f6-4923-964d-b0a1a635abf3" />


## 2. Prerequisites

*   ROS 2 (e.g., Humble Hawksbill or later) installed and sourced.
*   Gazebo (Ignition Gazebo) installed.
*   A ROS 2 workspace (e.g., `bumperbot_ws`).

## 3. Workspace Setup

1.  **Create ROS 2 Workspace:**
    If you haven't already, create a ROS 2 workspace:
    ```bash
    mkdir -p ~/SharedDrive/Harris/Desktop/ros2_workspaces/bumperbot_ws/src
    cd ~/SharedDrive/Harris/Desktop/ros2_workspaces/bumperbot_ws/
    colcon build --symlink-install
    source install/setup.bash
    ```
    *(Note: The `colcon build` command is typically run after creating packages, but it's good practice to have the workspace structure ready.)*

2.  **Place Package:**
    Ensure the `bumperbot_description` package is located within the `src` directory of your workspace:
    `/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/bumperbot_ws/src/bumperbot_description/`

3.  **Build the Workspace:**
    Navigate to your workspace root and build:
    ```bash
    cd ~/SharedDrive/Harris/Desktop/ros2_workspaces/bumperbot_ws/
    colcon build --symlink-install
    ```

4.  **Source the Workspace:**
    After building, source your workspace's setup file:
    ```bash
    source install/setup.bash
    ```
    This makes the `bumperbot_description` package and its executables available to ROS 2.

## 4. Understanding the `bumperbot_description` Package

This package is crucial for defining and simulating your robot. It typically contains:

*   **URDF/XACRO Files (`urdf/`):**
    *   `bumperbot.urdf.xacro`: Defines the robot's physical structure, links, joints, and their relationships. It uses XACRO for macros and properties.
    *   `bumperbot.gazebo.xacro`: Extends the URDF to include Gazebo-specific properties like physics, sensors, and visual elements.
*   **Meshes (`meshes/`):**
    *   3D model files (e.g., `.STL`) used for the visual representation of robot parts in Gazebo and Rviz.
*   **Launch Files (`launch/`):**
    *   `gazebo.launch.py`: A Python script to launch the Gazebo simulation environment and spawn the robot.
    *   `display.launch.py`: (Likely) A launch file to display the robot model in Rviz.
*   **Rviz Configuration (`rviz/`):**
    *   `bumperbot.rviz`: A configuration file for Rviz, pre-set to display the robot model and potentially other topics.

## 5. Essential ROS 2 Packages Used

The following packages are declared as `exec_depend` in the `package.xml` and are essential for this project:

*   **`robot_state_publisher`**:
    *   **Purpose**: This node takes the robot's joint states (from topics like `/joint_states`) and publishes their corresponding transformations (TF) to the TF tree. This allows other nodes and visualization tools to know the position and orientation of each part of the robot in 3D space.
    *   **Usage**: It's crucial for visualizing the robot's kinematic chain in Rviz and for any system that relies on the robot's pose.

*   **`joint_state_publisher`**:
    *   **Purpose**: This node typically publishes default or manually controlled joint states. In simulation, it's often used to provide initial joint positions or to allow for interactive control of the robot's joints for debugging and visualization in Rviz.
    *   **Usage**: Used in conjunction with `robot_state_publisher` and Rviz to display the robot with specific joint configurations.

*   **`rviz`**:
    *   **Purpose**: RViz (ROS Visualization) is a powerful 3D visualization tool for ROS. It allows you to visualize sensor data (like point clouds, images, laser scans), robot models, TF frames, and more.
    *   **Usage**: You'll use Rviz to view the Bumperbot model, its current state, and any simulated sensor data. The `bumperbot.rviz` file provides a pre-configured setup.

*   **`ros2launch`**:
    *   **Purpose**: This is the command-line tool used to execute ROS 2 launch files (`.launch.py`, `.launch.xml`). Launch files are scripts that define and start multiple ROS 2 nodes, set parameters, and configure the ROS graph.
    *   **Usage**: You will use `ros2 launch bumperbot_description gazebo.launch.py` to start the simulation.

*   **`ros_gz_sim`**:
    *   **Purpose**: This package provides the ROS 2 interface for Ignition Gazebo (now often referred to as Gazebo). It allows ROS 2 nodes to interact with the Gazebo simulator, such as spawning models, controlling physics, and receiving sensor data.
    *   **Usage**: It's used in the `gazebo.launch.py` file to start the Gazebo simulator (`gz_sim.launch.py`) and to spawn the Bumperbot model into the simulation world (`ros_gz_sim create` node).

## 6. Launching the Robot in Gazebo

To launch the Bumperbot simulation in Gazebo:

1.  **Ensure your workspace is sourced:**
    ```bash
    source ~/SharedDrive/Harris/Desktop/ros2_workspaces/bumperbot_ws/install/setup.bash
    ```

2.  **Run the Gazebo launch file:**
    ```bash
    ros2 launch bumperbot_description gazebo.launch.py
    ```

This command will:
*   Start the `robot_state_publisher` to make the robot's URDF available.
*   Set up the necessary environment variables for Gazebo.
*   Launch the Gazebo simulator.
*   Spawn the Bumperbot model into the Gazebo world using the URDF defined in `urdf/bumperbot.urdf.xacro` and its Gazebo extensions.

You should see the Gazebo simulator window open with the Bumperbot robot loaded. You can then use Rviz (potentially with another launch file like `display.launch.py`) to visualize the robot model.
