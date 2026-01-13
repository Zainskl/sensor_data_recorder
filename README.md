# Sensor Data Recorder ROS2
This repository is a sample project for recording data from different ros2 nodes, preferably sensor data being published via topics.

# Features
- Configurable code
- mcap format for foxglove web view (easy to change to sqlite3)
- Record Either Specific topics via providing names or using --all flag

# Requirements


# Usage
1. Download src folder to your workspace or use the repository folder as ros2 workspace
2. Open two terminals preferably

*[CMD]*
```
colcon build
ros2 launch sdr sdr.launch.py
ros2 run sensor_data_publisher fake_sensor.py
```

# Note
* SDR is Sensor Data Recording Package
* Sensor_Data_Publisher is used for producing mock sensor data to test the data recording package.

