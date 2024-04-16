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
  - `float64[] samples`: Array of sensor data samples.
  - `bool success`: Indicates whether the data was successfully fetched.

## Usage
To use the `GetSensorData` service, follow these steps:
1. Ensure your ROS2 environment is setup and sourced correctly.
2. Navigate to your `ros2_ws` workspace directory.
3. Execute the service call:
   ```bash
   ros2 service call /get_sensor_data robotic_soln_interfaces/srv/GetSensorData "{'num_samples': 1000}"
