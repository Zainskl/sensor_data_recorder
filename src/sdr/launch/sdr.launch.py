#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from datetime import datetime
import os

try:
    HOME_DIR = os.path.expanduser("~")
except RuntimeError:
    HOME_DIR = "/tmp"  # Fallback to /tmp if home resolution fails

SENSOR_LOG_DIR = "sensor_logs"
LOG_PATH = os.path.join(HOME_DIR, SENSOR_LOG_DIR)

os.makedirs(LOG_PATH, exist_ok=True)

def generate_launch_description():
    
    bag_file_name = datetime.now().strftime('%y-%m-%d_%H%M%S') #For Unique Filename
    
    output_file = os.path.join(LOG_PATH, bag_file_name) 
    
    sensor_recorder = ExecuteProcess(
        cmd = [
            "ros2", "bag", "record",
            "--storage", "mcap", # mcap for foxglove studio, sqlite3 for plotjuggler
            "--max-cache-size", "0",
            "/topic_sensor1", "topic_sensor2",
            "-o", output_file
        ],
        output="screen"
    )

    return LaunchDescription([
        TimerAction(
            period=5.0,
            actions=[sensor_recorder]
        )
    ])
