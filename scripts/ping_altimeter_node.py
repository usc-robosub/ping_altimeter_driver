#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from brping import Ping1D
import time
from geometry_msgs.msg import Pose

class Ping1DAltimeterNode:
    def __init__(self):
        # Parameters
        self.port = rospy.get_param('~port', '/dev/serial0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'ping1d')
        self.rate = rospy.get_param('~rate', 10)  # Hz

        # Initialize Ping1D device
        self.ping = Ping1D()
        rospy.loginfo("Connecting to Ping1D on %s at %d bps...", self.port, self.baudrate)
        self.ping.connect_serial(self.port, self.baudrate)

        if not self.ping.initialize():
            rospy.logerr("Failed to initialize Ping1D!")
            rospy.signal_shutdown("Ping1D device not responding.")
            return

        rospy.loginfo("Ping1D initialized successfully.")

        # ROS publisher
        self.pub = rospy.Publisher('ping1d/range', Range, queue_size=10)
        self.pose_pub = rospy.Publisher('ping1d/pose', Pose, queue_size=10)

        # Start polling loop
        self.run()

    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            data = self.ping.get_distance()
            if data:
                msg = Range()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.1  # Set as needed
                msg.min_range = 0.02     # Ping1D minimum in meters
                msg.max_range = 30.0     # Ping1D max range (adjust as needed)
                msg.range = data['distance'] / 1000.0  # Convert mm to m

                msg2 = Pose
                msg2.header.stamp = rospy.Time.now()
                msg2.header.frame_id = self.frame_id
                msg2.position.x = 0.0
                msg2.position.y = 0.0
                msg2.position.z = data['distance'] / 1000.0

                self.pub.publish(msg)
                self.pose_pub.publish(msg2)
            else:
                rospy.logwarn("Failed to read distance from Ping1D.")
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('ping1d_altimeter_node')
    try:
        Ping1DAltimeterNode()
    except rospy.ROSInterruptException:
        pass
