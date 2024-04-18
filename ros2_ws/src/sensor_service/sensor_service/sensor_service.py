#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from robotic_soln_interfaces.srv import GetSensorData

class SensorService(Node):

    def __init__(self):
        super().__init__('sensor_service')
        self.srv = self.create_service(
            GetSensorData, 'get_sensor_data', self.handle_sensor_data_request)
        self.latest_data = np.array([]) # Initialize empty array to store sensor data

    def read_sensor_data(self, num_samples):
        return np.random.rand(3, num_samples)
        
        