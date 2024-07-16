
# Project README

## Project Overview
This project involves simulating a robotic hand using the MuJoCo physics engine. The main goals are to control the robotic hand's joints to grasp an object using PID control and impedance control, lift the object to a specified height, and visualize the torque data during these processes.

## Requirements

### Installation
To set up and run the project, you need to install the following packages:
1. **MuJoCo** - A physics engine for simulating articulated structures.
2. **NumPy** - A fundamental package for numerical operations in Python.
3. **Matplotlib** - A plotting library for creating static, animated, and interactive visualizations in Python.

You can install the required packages using the following commands:
```sh
pip install mujoco-py numpy matplotlib
```

Additionally, you need to have MuJoCo properly installed on your system. Follow the [MuJoCo installation instructions](http://www.mujoco.org/book/install.html).

### Project Structure
The project consists of the following main components:
1. **Main Simulation Script (`main.py`)**
   - Loads the MuJoCo model.
   - Initializes the viewer.
   - Sets up joint goals and sensors.
   - Runs PID and impedance control loops.
   - Plots the torque data.
   
2. **Sensor Utilities (`sensor.py`)**
   - Functions to get sensor IDs, read and smooth sensor data, and check for contacts in the simulation.

3. **Utility Functions (`utils.py`)**
   - Helper functions to get body names and positions, print all bodies, and get joint and actuator IDs.

4. **Control Functions (`controller.py`)**
   - Implements PID control for joint movements.
   - Implements impedance control based on contact and sensor data.
   - Lifts an object to a target position.

## Features

### PID Control
- **Function**: `pid_control`
- **Description**: Controls the joint movements to reach target positions using Proportional-Integral-Derivative (PID) control.
- **Parameters**:
  - `model`: The MuJoCo model.
  - `data`: The MuJoCo data.
  - `joint_ids`: Dictionary of joint IDs.
  - `actuator_ids`: Dictionary of actuator IDs.
  - `position_goals`: Target positions for each joint.
  - `integral_errors`: Integral errors for each joint.
  - `kp`, `ki`, `kd`: PID gains.

### Impedance Control
- **Function**: `impedance_control`
- **Description**: Adjusts the joint torques based on contact information and torque sensors to maintain a compliant grasp.
- **Parameters**:
  - `model`: The MuJoCo model.
  - `data`: The MuJoCo data.
  - `joint_ids`: Dictionary of joint IDs.
  - `contact_info`: Information about contacts between the robotic hand and the object.
  - `torque_sensors`: Dictionary of torque sensor IDs.
  - `sensor_data_buffer`: Buffer for smoothing sensor data.
  - `window_size`: Size of the window for smoothing data.
  - `ctrlrange`: Control range for the actuators.
  - `kp`, `kd`, `ki`: Impedance control gains.

### Lifting the Object
- **Function**: `lift_object`
- **Description**: Controls the lifting of the object to a specified z position.
- **Parameters**:
  - `model`: The MuJoCo model.
  - `data`: The MuJoCo data.
  - `ctrlrange`: Control range for the actuators.
  - `z_goal`: Target z position for lifting the object.
  - `kp`, `kd`: PID gains for lifting.

### Contact Checking
- **Function**: `check_contact`
- **Description**: Checks for contacts between specified palm joints and the object.
- **Parameters**:
  - `model`: The MuJoCo model.
  - `data`: The MuJoCo data.
  - `object_body_name`: Name of the object body.
  - `palm_joint_names`: List of palm joint names.

### Data Smoothing
- **Function**: `smooth_data`
- **Description**: Smooths sensor data using a moving average.
- **Parameters**:
  - `sensor_data`: The raw sensor data.
  - `key`: Key to identify the sensor data.
  - `sensor_data_buffer`: Buffer for the sensor data.
  - `window_size`: Size of the window for smoothing data.

## Output
The main output of the project is a plot of the real-time torque data during the grasping process with impedance control. The plot is saved as a PNG file (`torque_data(Impedance Control).png`) and shows the torque values over time for each sensor.

### Example Output
![Real-time Torque Data During Grasping with Impedance Control](torque_data(Impedance Control).png)

### Running the Simulation
To run the simulation, execute the `main.py` script:
```sh
python main.py
```
This will start the simulation, apply the control strategies, and generate the torque data plot.

## Conclusion
This project demonstrates the use of PID and impedance control for robotic hand simulation using MuJoCo. By following the installation and usage instructions, you can reproduce the simulation and analyze the torque data generated during the grasping process.
