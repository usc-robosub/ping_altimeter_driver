#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
import yaml
import os
import time

try:
    from brping import Ping1D
except ImportError:
    raise ImportError("Please install the ping-python package: pip install ping-python")

def load_config():
    config_path = rospy.get_param('~config', os.path.join(
        os.path.dirname(__file__), '../config/ping_altimeter.yaml'))
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rospy.init_node('ping_altimeter_driver')
    config = load_config()

    serial_port = rospy.get_param('~serial_port', config['serial_port'])
    frame_id = rospy.get_param('~frame_id', config['frame_id'])
    frequency = rospy.get_param('~frequency', config['frequency'])
    fov = rospy.get_param('~fov', config['fov'])
    min_range = rospy.get_param('~min_range', config['min_range'])
    max_range = rospy.get_param('~max_range', config['max_range'])

    pub = rospy.Publisher('ping_altimeter/range', Range, queue_size=10)

    sonar = Ping1D()
    if not sonar.connect_serial(serial_port, 115200):
        rospy.logerr(f"Failed to connect to Ping device on {serial_port}")
        return
    rospy.loginfo(f"Connected to Ping device on {serial_port}")

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        data = sonar.get_distance()
        if data and data['distance'] is not None:
            msg = Range()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = fov
            msg.min_range = min_range
            msg.max_range = max_range
            msg.range = data['distance'] / 1000.0  # mm to meters
            pub.publish(msg)
        else:
            rospy.logwarn("No data from Ping device.")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
