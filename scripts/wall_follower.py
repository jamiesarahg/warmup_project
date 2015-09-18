#!/usr/bin/env python
"""This is code for a wall following neato"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class WallFollower(object):
	def __init__(self):
		rospy.init_node('wall follower')

		self.error = 0
		self.gain = 1
		self.target_distance = 2

		self.twist = Twist()
		self.scan = LaserScan()
		rospy.Subscriber("/scan", LaserScan, self.wallErrorCalc)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


	def run(self):
		pass

if __name__ == '__main__':
  run = WallFollower()
  run.run()