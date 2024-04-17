# sensor_service - ROS2 Service Package Overview

## Overview
The `sensor_service` package is designed to interact with a TCP server running a sensor simulation (`sensor.py`). It provides a ROS2 service that fetches data from the sensor server and makes it available to ROS2 clients. This package is for applications that require real-time sensor data in a ROS2 environment.

## Dependencies
- **Python:** Python 3.8 or newer is required.
- **sensor.py:** This package depends on `sensor.py` running as a TCP server to function correctly. Ensure that `sensor.py` is set up and running before starting the ROS2 services provided by this package.
- **robotic_soln_interfaces:** This package is necessary to include as it includes the custom service for the 3-DOF sensor.

## Setup and Running
### Starting the Sensor Server
1. Navigate to the directory containing `sensor.py`.
2. Run the script using Python:
   ```bash
   python3 sensor.py

   ```
   This will start the TCP server that simulates the sensor.
3. Open a new terminal and ensure that your ROS2 environment is sourced:
   ```bash
   source install/setup.bash