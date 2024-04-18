# robotic_soln_interfaces - ROS2 Package Overview

## Overview
The `robotic_soln_interfaces` package provides ROS2 service interfaces for interacting with 3-DOF sensors. It includes services that facilitate the fetching of sensor data, designed to be scalable and adaptable to meet various application requirements.

## Services
### GetSensorData
- **Type:** Service
- **Description:** Allows clients to request a specified number of data samples from a 3-DOF sensor.
- **Request Parameters:**
  - `int32 num_samples`: Number of samples to fetch.
- **Response Parameters:**
  - `float64[] samples`: Array of sensor data samples. Since it's 3-DOF, this is grouping all axes together, but we could easily separate this into 3 arrays to collect for each axis. 
  - `bool success`: Indicates whether the data was successfully fetched.

## Testing
To test the `GetSensorData` service is implemented correctly, you can check in the directory of the `ros2_ws` with the following commands:
```bash
  cd Robotic_soln/ros2_ws/
  colcon build --packages-select robotic_soln_interfaces
  source install/setup.bash
  ros2 interface show robotic_soln_interfaces/srv/GetSensorData 
```

If implemented/setup correctly, you should expect to see the following in the terminal in response to the previous commands:
```bash
# Request message
int32 num_samples
---
# Response message
float64[] sensor_data
bool success
```