#!/usr/bin/env python3
""" Simple client that calls the same service twice and publishes combined results to a topic at 500 Hz. """
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from robotic_soln_interfaces.srv import GetSensorData
from std_msgs.msg import Float64MultiArray
import numpy as np

class SensorClient(Node):

    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(GetSensorData, 'get_sensor_data', callback_group=ReentrantCallbackGroup())
        self.publisher = self.create_publisher(Float64MultiArray, 'sensor_data', 10)
        self.timer = self.create_timer(0.002, self.timer_callback)  # 500Hz frequency
        self.pending_responses = []
        self.get_logger().info("Client started, ready to make requests.")

    def send_request(self):
        if self.client.wait_for_service(timeout_sec=1.0):
            request = GetSensorData.Request()
            request.num_samples = 10
            future = self.client.call_async(request)
            self.pending_responses.append(future)
            self.get_logger().info("Request sent to service.")
        else:
            self.get_logger().warn("Service not available. Retrying...")

    def timer_callback(self):
        self.send_request()
        for future in list(self.pending_responses):
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        data = Float64MultiArray()
                        data.data = response.sensor_data
                        self.publisher.publish(data)
                        self.get_logger().info(f"Published data: {data.data}")
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                self.pending_responses.remove(future)

def main(args=None):
    rclpy.init(args=args)
    sensor_client = SensorClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(sensor_client, executor=executor)
    sensor_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
