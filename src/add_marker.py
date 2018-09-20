#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import rospy

class AddMarker(object):
    """This node publishes a marker to rviz"""

    def __init__(self):
        rospy.init_node("AddMarker")
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        #.Subscriber('/bump', Bump, self.process_bump)
        #rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_bump(self, m):
        pass
        #print(m.leftFront)

    def draw_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.SPHERE
        marker.pose.position.x = 1
        marker.pose.position.y = 2
        marker.pose.position.z = 0
        marker.scale.x = 1
	marker.scale.y = 1
	marker.scale.z = 1
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        self.pub.publish(marker)

    def run(self):
        #r = rospy.Rate(2)

        while not rospy.is_shutdown():
            print('here')
            self.draw_marker()
            rospy.sleep(.01)

if __name__ == '__main__':
    node = AddMarker()
    node.run()
