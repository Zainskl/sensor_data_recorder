#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header, Float64
import random
import sys
import logging # for saving logs of errors occuring outside of ros2

NODE_NAME = "Fake_Sensor_Publisher"

pylogger = logging.getLogger(NODE_NAME)
logging.basicConfig(
    level=logging.INFO,  # Set default logging level
    format=f"%(asctime)s - %(levelname)s - [%(name)s] - %(message)s",
    datefmt="%H:%M:%S",
    handlers= [
        logging.StreamHandler(sys.stdout)
    ]
)
# Above configuration along with Node_Name is only for python logging & Debugging the system outside of ros2
# For Ros2 native logging, we use ros2 builtin logging system

class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')
        
        # Create publishers for two sensors
        self.sensor1_pub = self.create_publisher(Float64, 'topic_sensor1', 10)
        self.sensor2_pub = self.create_publisher(Float64, 'topic_sensor2', 10)
        
        # Timer to publish sensor data every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Fake Sensor Publisher Node Started')
        self.get_logger().info('Publishing every 0.5 seconds on topics:')
        self.get_logger().info('  - /sensor1_data (std_msgs/Float64)')
        self.get_logger().info('  - /sensor2_data (std_msgs/Float64)')

    def timer_callback(self):
        # Publish Sensor 1 data
        self.publish_sensor1()
        
        # Publish Sensor 2 data
        self.publish_sensor2()

    def publish_sensor1(self):
        msg = Float64()
        # Generate random number between 0 and 100
        msg.data = random.uniform(0.0, 100.0)
        
        self.sensor1_pub.publish(msg)
        self.get_logger().info(f'Sensor 1: {msg.data:.2f}')

    def publish_sensor2(self):
        msg = Float64()
        # Generate random number between 0 and 100
        msg.data = random.uniform(0.0, 100.0)
        
        self.sensor2_pub.publish(msg)
        self.get_logger().info(f'Sensor 2: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node) 
    except Exception as e:
        pylogger.error(f"An error occurred: {str(e)}")
    finally:
        if rclpy.ok():
            pylogger.warning("Calling Node Shutdown!")
            rclpy.shutdown()

if __name__ == '__main__':
    main()
