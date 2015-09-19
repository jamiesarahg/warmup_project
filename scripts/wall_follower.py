#!/usr/bin/env python
"""This is code for a wall following neato"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class FiniteStateMachine(object):
	def __init__(self):
		rospy.init_node('wallfollower')
		
		rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		rospy.Subscriber("/odom", Odometry, self.odomCallback)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.twist = Twist()
		self.scan = LaserScan()

		self.ranges = False
		self.odom = False


		self.status = 'Start'
		self.finder = WallFinder()
		self.follower = WallFollower()

		self.angletolerance = 2
		self.finder.angletolerance = self.angletolerance
	
		self.distancetolerance = .1
		self.finder.distancetolerance = self.distancetolerance



	def run(self):
		print self.status
		if self.status == 'Start':
			if self.odom and self.ranges:
				self.status = 'FaceWall'
		if self.status == 'FaceWall':
			if self.finder.faceDone:
				self.status = 'DistanceWall'
			else:
				self.finder.ranges = self.ranges
				self.finder.odom = self.odom
				self.finder.findWall(0)
				self.twist.angular.z = self.finder.error * -.01
				self.twist.linear.x = 0
				self.pub.publish(self.twist)
		if self.status == 'DistanceWall':
			if self.finder.distanceDone:
				self.twist.linear.x = 0
				self.pub.publish(self.twist)
				self.status = 'Rotate'
				self.finder.faceDone = False
			else:
				self.finder.ranges = self.ranges
				self.finder.odom = self.odom
				self.finder.gotoWall()
				self.twist.linear.x = self.finder.error
				self.pub.publish(self.twist)
		if self.status == 'Rotate':
			if self.finder.faceDone:
				self.status = 'FollowWall'
			else:
				self.finder.ranges = self.ranges
				self.finder.odom = self.odom
				self.finder.findWall(90)
				self.twist.angular.z = self.finder.error * -.01
				self.twist.linear.x = 0
				self.pub.publish(self.twist)




	def scanCallback(self, data):
		self.ranges = data.ranges
	
	def odomCallback(self,data):
		shortcut = data.pose.pose
		orientation_tuple = (shortcut.orientation.x, shortcut.orientation.y, shortcut.orientation.z, shortcut.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.odom = shortcut.position.x, shortcut.position.y, angles[2]

class WallFinder(FiniteStateMachine):
	def __init__(self):
		self.distanceDone = False
		self.ranges = False
		self.odom = False
		self.target_distance = .5
		self.faceDone = False

	def findWall(self, orient):
		try: minimum_distance = min(filter(lambda a: a!=0, self.ranges))
		except ValueError:
			return
		print "min dist ", minimum_distance
		print "odom ", self.odom[2]
		ind = self.ranges.index(minimum_distance)
		print "ind - orient ", ind-orient
		self.error = self.angle_diff(self.odom[2], ind-orient) 
		print "error", self.error
		if abs(self.error)> self.angletolerance:
			self.error = self.angle_diff(self.odom[2], ind-orient)
		else:
			self.faceDone = True

	def gotoWall(self):
		self.error = self.ranges[0] - self.target_distance
		if self.error < self.distancetolerance:
			self.distanceDone = True


	def angle_diff(self, a, b):
		""" Calculates the difference between angle a and angle b (both should be in radians)
		the difference is always based on the closest rotation from angle a to angle b
		examples:
		angle_diff(.1,.2) -> -.1
		angle_diff(.1, 2*math.pi - .1) -> .2
		angle_diff(.1, .2+2*math.pi) -> -.1
		"""

		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1 > 0:
			d2 *= -1.0
		if math.fabs(d1) < math.fabs(d2):
			return d1
		else:
			return d2


class WallFollower(FiniteStateMachine):
	def __init__(self):
		# rospy.init_node('wallfinder')
		pass
		# if (x >= self.target_min and x <= self.target_max):
		# 	print "following"
		# 	self.ang_error = y_backward - y_forward
		# 	self.twist.angular.z = self.gain*self.ang_error
		# else:
			# y_backward = self.ranges[135]

		# 	x = self.ranges[90]
		# y_forward = self.ranges[45]
		

		
		


if __name__ == '__main__':
	run = FiniteStateMachine()
	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		run.run()
		r.sleep()



		# self.lin_error = 0
		# self.ang_error = 0
		# self.gain = 1
		# self.target_distance = 1
		# self.allowed_error = .2
		# self.target_max = self.target_distance + self.allowed_error
		# self.target_min = self.target_distance - self.allowed_error
		# self.angle_distance = self.target_distance * math.sqrt(2)
		# self.twist = Twist()
		# self.scan = LaserScan()
		

		# rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		# rospy.Subscriber("/odom", Odometry, self.odomCallback)
		# print "subscribed"


		# self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)