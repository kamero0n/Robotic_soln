#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from robotic_soln_interfaces.srv import GetSensorData

class SensorService(Node):

    def __init__(self):
        super().__init__('service')
        self.srv = self.create_service(GetSensorData, 'get_sensor_data', self.handle_sensor_data_request)
        self.latest_data = np.array([])  # Initialize empty array to store sensor data
        self.get_logger().info("Service has started and is ready to handle requests.")

    def read_sensor_data(self, num_samples):
        data = np.random.rand(3, num_samples)
        self.get_logger().info(f"Generated sensor data: {data}")
        return data
    
    def handle_sensor_data_request(self, request, response):
        self.get_logger().info(f"Received request for {request.num_samples} samples.")
        try:
            num_samples = request.num_samples
            if num_samples > 0:
                response.sensor_data = self.read_sensor_data(num_samples).flatten().tolist()
                response.success = True
                self.get_logger().info(f"Sending sensor data: {response.sensor_data}")
            else:
                response.success = False
                response.sensor_data = []
                self.get_logger().warn("Request for zero or negative number of samples.")
        except Exception as e:
            response.success = False
            response.sensor_data = []
            self.get_logger().error(f"Failed to process request: {str(e)}")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    sensor_service = SensorService()
    rclpy.spin(sensor_service)
    sensor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
