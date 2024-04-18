#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robotic_soln_interfaces.srv import GetSensorData
from std_msgs.msg import Float64MultiArray

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'sensor_data', 10)
        self.sensor1_client = self.create_client(GetSensorData, 'get_sensor_data1')
        self.sensor2_client = self.create_client(GetSensorData, 'get_sensor_data2')

        self.sensor1_request = GetSensorData.Request()
        self.sensor2_request = GetSensorData.Request()
        self.sensor1_request.num_samples = 100
        self.sensor2_request.num_samples = 100

        # Using a lower frequency to start requests to avoid spamming unresponsive services
        self.timer = self.create_timer(1.0, self.publish_sensor_data)  # Request data every second initially

    def publish_sensor_data(self):
        # Non-blocking service calls
        if not self.sensor1_client.wait_for_service(timeout_sec=1.0) or not self.sensor2_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("One or both services are not available.")
            return

        sensor1_future = self.sensor1_client.call_async(self.sensor1_request)
        sensor2_future = self.sensor2_client.call_async(self.sensor2_request)
        sensor1_future.add_done_callback(self.handle_service_response)
        sensor2_future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                data = Float64MultiArray()
                data.data = response.sensor_data
                self.publisher.publish(data)
                self.get_logger().info("Data published successfully.")
            else:
                self.get_logger().error("Service response not successful.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
