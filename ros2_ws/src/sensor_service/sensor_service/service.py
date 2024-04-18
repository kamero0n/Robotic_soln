#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from robotic_soln_interfaces.srv import GetSensorData

class SensorService(Node):

    def __init__(self):
        super().__init__('service')
        self.srv = self.create_service(
            GetSensorData, 'get_sensor_data', self.handle_sensor_data_request)
        self.socket_setup()

    def socket_setup(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('127.0.0.3', 10000)
        self.sock.connect(server_address)
        self.get_logger().info(f"Connected to server at {server_address}")

    def read_sensor_data(self, num_samples):
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock.sendall(message)
        byte_data = self.sock.recv(10000 * num_samples)
        sensor_data = np.frombuffer(byte_data, dtype=np.float64)
        return sensor_data
    
    def handle_sensor_data_request(self, request, response):
        self.get_logger().info(f"Received request for {request.num_samples} samples.")
        try:
            response.sensor_data = self.read_sensor_data(request.num_samples).tolist()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor data: {e}")
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
