# Bumperbot Controller

This package contains the ROS 2 controllers for the Bumperbot robot.

## Project Explanation and Notes

This section will detail the steps involved in setting up and understanding the bumperbot_controller package. It will also serve as a personal reference for future development.

### 1. Overview
*   **Purpose:** The `bumperbot_controller` package is responsible for controlling the various joints and actuators of the Bumperbot robot within the ROS 2 framework.
*   **Technologies:** It utilizes `ros2_control` for hardware abstraction and controller management.

### 2. Setup (Notes)
*   Ensure `ros2_control` and its dependencies are installed.
*   This package typically works in conjunction with `bumperbot_description` (for URDF/XACRO definition) and `bumperbot_bringup` (for launching the robot).

### 3. Key Files and Directories
*   `config/`: Contains YAML configuration files for the controllers.
*   `src/`: C++ or Python source files for custom controllers or nodes.
*   `CMakeLists.txt`/`package.xml`: Standard ROS 2 package files.

### 4. How to Use/Launch
*   Compile the workspace: `colcon build --packages-select bumperbot_controller`
*   Source the workspace: `. install/setup.bash` (or your preferred shell setup script)
*   Launch the robot (example, actual command may vary):
    ```bash
    ros2 launch bumperbot_bringup bumperbot_launch.py
    ```

### 5. Testing the Robot Setup

To quickly test if the velocity controller is working correctly, you can publish a command to its topic:

```bash
ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0, 0.0]}"
```
This command sends a velocity of 5.0 units to the first joint and 0.0 to the second joint. Observe the robot's behavior in simulation or on hardware.

### 6. Controlling the Robot with Inverse Kinematics (simple_controller.cpp/py)

The `simple_controller.cpp` (and its Python equivalent `simple_controller.py`) nodes implement inverse kinematics to control the robot's linear (`vx`, `vy`) and angular (`theta_dot`) velocities by converting them into individual wheel velocities. This allows for more intuitive control of the robot's motion.

To control the robot using these nodes, publish `geometry_msgs/msg/TwistStamped` messages to the `/bumperbot_controller/cmd_vel` topic:

```bash
ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

This command will set a linear velocity of 0.2 m/s in the x-direction and an angular velocity of 0.5 rad/s around the z-axis.

## ROS 2 Control CLI Commands

These commands are useful for inspecting and managing `ros2_control` components.

*   **List all available controllers:**
    ```bash
    ros2 control list_controllers
    ```

*   **List all hardware components (e.g., Robot Hardware interfaces):**
    ```bash
    ros2 control list_hardware_components
    ```

*   **Load and start a controller (example: `joint_state_broadcaster`):**
    ```bash
    ros2 control load_controller joint_state_broadcaster
    ros2 control set_controller_state joint_state_broadcaster start
    ```

*   **Stop and unload a controller:**
    ```bash
    ros2 control set_controller_state joint_state_broadcaster stop
    ros2 control unload_controller joint_state_broadcaster
    ```

*   **List controller types:**
    ```bash
    ros2 control list_controller_types
    ```

*   **Detailed information about a specific controller (if available/running):**
    ```bash
    ros2 topic info /controller_name/topic_name
    ```
    (Replace `/controller_name/topic_name` with actual topic for state or commands)

## Note on Forward Differential Kinematics for a Differential Drive Robot

Forward Differential Kinematics relates the wheel velocities to the robot's chassis velocity. For a differential drive robot, this is expressed using the Jacobian matrix.

Given:
*   `v_e`: The robot's chassis velocity vector, `[vx, vy, θ_dot]^T`, where `vx` is the linear velocity along the robot's forward direction, `vy` is the linear velocity sideways, and `θ_dot` is the angular velocity. For a standard non-holonomic differential drive robot, `vy` is typically constrained to be 0.
*   `q_dot`: The wheel angular velocity vector, `[φ_dot_left, φ_dot_right]^T`, where `φ_dot_left` and `φ_dot_right` are the angular velocities of the left and right wheels, respectively.
*   `r`: The radius of the wheels.
*   `l`: The separation distance between the two wheels.

The relationship is:

`v_e = J * q_dot`

Which expands to:

$$
\begin{bmatrix} v_x \\ v_y \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \frac{r cos(\phi)}{2} & \frac{r cos(\phi)}{2} \\ 
    \frac{r sin(\phi)}{2} & \frac{r sin(\phi)}{2}  \\ 
    \frac{r}{l} & -\frac{r}{l} 
\end{bmatrix} 
\begin{bmatrix} \dot{\phi}_{left} \\ \dot{\phi}_{right} \end{bmatrix}
$$


The Jacobian matrix `J` for this system is:

$$
J = \begin{bmatrix} \frac{r cos(\phi)}{2} & \frac{r cos(\phi)}{2} \\ 
    \frac{r sin(\phi)}{2} & \frac{r sin(\phi)}{2}  \\ 
    \frac{r}{l} & -\frac{r}{l} 
\end{bmatrix} 
$$

This formulation is crucial for calculating the robot's movement from wheel encoder data (odometry) and for determining the required wheel velocities to achieve a desired chassis velocity (inverse kinematics).
