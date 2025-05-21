# ping_altimeter_driver

A ROS package for interfacing with the Blue Robotics Ping Altimeter using the [ping-python](https://github.com/bluerobotics/ping-python) library. This driver reads sonar data from the Ping device and publishes it as a `sensor_msgs/Range` message.

## Features
- Publishes range data from the Ping altimeter as a ROS topic
- Configurable via YAML file (serial port, frame_id, frequency, FOV, min/max range)
- Easy integration with other ROS nodes

## Directory Structure
ping_altimeter_driver/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── ping_altimeter.yaml
├── launch/
│   └── ping_altimeter.launch
├── scripts/
│   └── ping_altimeter_node.py

## Installation
1. **Install ROS dependencies:**
   ```bash
   sudo apt-get install ros-<distro>-rospy ros-<distro>-sensor-msgs ros-<distro>-std-msgs ros-<distro>-dynamic-reconfigure
   ```
   Replace `<distro>` with your ROS version (e.g., `noetic`).

2. **Install ping-python:**
   ```bash
   git clone https://github.com/bluerobotics/ping-python.git
   cd ping-python
   pip3 install .
   ```

## Configuration
Edit `config/ping_altimeter.yaml` to set:
- `serial_port`: Serial port for the Ping device (e.g., `/dev/ttyUSB0`)
- `frame_id`: Frame ID for the published message
- `frequency`: Publishing frequency in Hz
- `fov`: Field of view in radians
- `min_range`: Minimum range in meters
- `max_range`: Maximum range in meters

## Usage
To launch the node with the default configuration:
```bash
roslaunch ping_altimeter_driver ping_altimeter.launch
```

## Output
The node publishes `sensor_msgs/Range` messages on the topic:
```
/ping_altimeter/range
```

## License
MIT
