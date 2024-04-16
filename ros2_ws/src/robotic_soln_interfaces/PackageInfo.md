# Package Overview

This section provides an overview of the package within ros_ws.

## ' robotic_soln_interfaces'

The **robotic_soln_interfaces** provides interfaces and implementations for interacting with 3-DOF sensors. It currently includes a custom service for fetching sensor data. It can easily be modified. 

### Services
- **GetSensorData**
    - **Type:** Service
    - **Description:** This service allows clients to request a specified number of sensor data samples from a 3-DOF sensor.
    - **Request:**
        - **int32 number_of_samples:** The number of samples to fetch from the sensor
    - **Response:** 
        - **float64[] samples:** An array of sensor data samples.
        - **bool success:** Indicates whether the data was fetched succesfully

Usage Example
