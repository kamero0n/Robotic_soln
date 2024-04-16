#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from robotic_soln_interfaces.srv import GetSensorData

class SensorDataClient(Node):
    def __init__(self):
        super().__init__('sensor_data_client')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('127.0.0.3', 10000)
        self.client_socket.connect(self.server_address)
        self.srv = self.create_service(GetSensorData, 'get_sensor_data', self.handle_get_sensor_data)

    def handle_get_sensor_data(self, request, response):
        self.get_logger().info(f'Receiving request for {request.num_samples} samples')
        # Sending request to sensor server
        self.client_socket.sendall(str(request.num_samples).encode())

        # Receiving data
        try:
            byte_data = self.client_socket.recv(10000)  # Adjust buffer size as needed
            response.sensor_data = np.frombuffer(byte_data, dtype=np.float64).tolist()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to receive data: {str(e)}")
            response.sensor_data = []
            response.success = False

        return response

    def __del__(self):
        self.get_logger().info('Closing socket')
        self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    sensor_data_client = SensorDataClient()
    rclpy.spin(sensor_data_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
