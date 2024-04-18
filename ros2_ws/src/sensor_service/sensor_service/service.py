#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from robotic_soln_interfaces.srv import GetSensorData

class SensorService(Node):
    def __init__(self):
        super().__init__('service')
        # Initialize services for handling requests from two different sensors
        self.srv1 = self.create_service(GetSensorData, 'get_sensor_data1', self.handle_sensor_data_request1)
        self.srv2 = self.create_service(GetSensorData, 'get_sensor_data2', self.handle_sensor_data_request2)
        # Establish connections to the sensor hardware/simulation
        self.socket_setup()

    def socket_setup(self):
        # Connection setup for sensor 1
        self.sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address1 = ('127.0.0.3', 10000)
        self.sock1.connect(server_address1)
        self.get_logger().info(f"Connected to server 1 at {server_address1}")

        # Connection setup for sensor 2
        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address2 = ('127.0.0.1', 10000)
        self.sock2.connect(server_address2)
        self.get_logger().info(f"Connected to server 2 at {server_address2}")

    def read_sensor_data(self, sock, num_samples):
        # Send request for data and receive sensor data
        message_string = str(num_samples)
        message = message_string.encode()
        sock.sendall(message)
        byte_data = sock.recv(10000 * num_samples)
        sensor_data = np.frombuffer(byte_data, dtype=np.float64)
        # Return the processed sensor data
        return sensor_data
    
    def handle_sensor_data_request1(self, request, response):
        # Handle data requests from Sensor 1
        self.get_logger().info(f"Received request for {request.num_samples} samples from Sensor 1.")
        try:
            response.sensor_data = self.read_sensor_data(self.sock1, request.num_samples).tolist()
            response.success = True
            self.get_logger().info("Sensor data successfully sent back to client for Sensor 1.")
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor data from Sensor 1: {e}")
            response.success = False
        return response

    def handle_sensor_data_request2(self, request, response):
        # Handle data requests from Sensor 2
        self.get_logger().info(f"Received request for {request.num_samples} samples from Sensor 2.")
        try:
            response.sensor_data = self.read_sensor_data(self.sock2, request.num_samples).tolist()
            response.success = True
            self.get_logger().info("Sensor data successfully sent back to client for Sensor 2.")
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor data from Sensor 2: {e}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SensorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
