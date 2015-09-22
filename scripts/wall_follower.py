#!/usr/bin/env python
"""This is code for a wall following neato"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class FiniteStateMachine(object):
	def __init__(self, mode):
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
		self.person = PersonFollower()
		self.obstacle = ObstacleAvoider()

		self.angletolerance = 1
		self.finder.angletolerance = self.angletolerance
		self.follower.angletolerance = self.angletolerance
	
		self.distancetolerance = .05
		self.finder.distancetolerance = self.distancetolerance

		self.target_distance = .5
		self.finder.target_distance = self.target_distance
		self.follower.target_distance = self.target_distance

		self.mode = mode



	def run(self):
		print self.status
		if self.status == 'Start':
			if self.odom and self.ranges:
				self.status = 'FaceWall'
			else:
				return
		if self.mode == 'WallFollower':
			if self.status == 'FaceWall':
				if self.finder.faceDone:
					self.status = 'DistanceWall'
					self.finder.faceDone = False
				else:
					# self.finder.ranges = self.ranges
					# self.finder.odom = self.odom
					self.finder.findWall(0, self.ranges, self.odom)
					self.twist.angular.z = self.finder.error * -.01
					self.twist.linear.x = 0
					self.pub.publish(self.twist)
			if self.status == 'DistanceWall':
				if self.finder.distanceDone:
					self.twist.linear.x = 0
					self.pub.publish(self.twist)
					self.status = 'Rotate'
					self.finder.distanceDone = False
				else:
					# self.finder.ranges = self.ranges
					# self.finder.odom = self.odom
					self.finder.gotoWall(self.ranges)
					self.twist.linear.x = self.finder.error
					self.pub.publish(self.twist)
			if self.status == 'Rotate':
				if self.finder.faceDone:
					self.status = 'FollowWall'
					self.finder.faceDone = False
				else:
					# self.finder.ranges = self.ranges
					# self.finder.odom = self.odom
					self.finder.findWall(90)
					self.twist.angular.z = self.finder.error * -.01
					self.twist.linear.x = 0
					self.pub.publish(self.twist)
			if self.status == 'FollowWall':
				# self.follower.ranges = self.ranges
				# self.follower.odom = self.odom
				self.follower.checkFacing(self.ranges)
				if self.follower.ok == False:
					self.status = 'Start'
					print "start"
				else:
					self.follower.distanceCorrection(self.ranges)
					self.twist.angular.z = .5 * self.follower.follow_error
					self.twist.linear.x = .3
					self.pub.publish(self.twist)
		if self.mode == "PersonFollower":
			# self.finder.ranges = self.ranges
			# self.finder.odom = self.odom
			CoM, distance_away = self.person.calculateCenterOfMass(self.ranges)

			print CoM
			print distance_away
			self.twist.angular.z = CoM * .02
			self.twist.linear.x = distance_away * .1
			self.pub.publish(self.twist)

		if self.mode == "ObstacleAvoidance":
			twist_to_home, distance_away = self.obstacle.returnHome(self.odom)
			self.twist.linear.x = distance_away * .2
	
			total_force = self.obstacle.seeObstacle(self.ranges)
			self.twist.linear.x = .1
			self.twist.angular.z = -.03*total_force + twist_to_home * .7
			self.pub.publish(self.twist)

	def scanCallback(self, data):
		self.ranges = data.ranges
	
	def odomCallback(self,data):
		shortcut = data.pose.pose
		orientation_tuple = (shortcut.orientation.x, shortcut.orientation.y, shortcut.orientation.z, shortcut.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.odom = shortcut.position.x, shortcut.position.y, angles[2]

class WallFinder(object):
	def __init__(self):
		self.distanceDone = False
		# self.ranges = False
		self.odom = False
		self.faceDone = False

	def findWall(self, orient, ranges, odom):
		try: minimum_distance = min(filter(lambda a: a!=0, ranges))
		except ValueError:
			return
		print "min dist ", minimum_distance
		print "odom ", odom[2]
		ind = ranges.index(minimum_distance)
		print "ind - orient ", ind-orient
		self.error = self.angle_diff(odom[2], ind-orient) 
		print "error", self.error
		if abs(self.error)> self.angletolerance:
			self.error = self.angle_diff(odom[2], ind-orient)
		else:
			self.faceDone = True

	def gotoWall(self, ranges):
		print "range", ranges[0]
		print "target",self.target_distance
		self.error = ranges[0] - self.target_distance
		if abs(self.error) < self.distancetolerance:
			print 'set to true'
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


class WallFollower(object):
	def __init__(self):
		# self.ranges = False
		self.ok = True
		self.follow_tolerance = 3
		self.follow_error = False


	def checkFacing(self, ranges):
		x = ranges[90]
		y_forward = list(ranges)[70:90]
		y_forward = filter(lambda a: a!=0, y_forward)
		y_mean_forward = sum(y_forward)/len(y_forward)
		y_backward = list(ranges)[90:110]
		y_backward = filter(lambda a: a!=0, y_backward)
		y_mean_backward = sum(y_backward)/len(y_backward)


		forward_error = abs(abs(self.target_distance/math.cos(10)) - y_mean_forward)
		backward_error = abs(abs(self.target_distance/math.cos(10)) - y_mean_forward)


		print "forward error", forward_error
		print "backward error", backward_error

		if forward_error > self.follow_tolerance or backward_error > self.follow_tolerance:
			self.ok = False
		else:
			self.ok = True

	def distanceCorrection(self, ranges):

		self.follow_error = ranges[90] - self.target_distance
		print "90 error", self.follow_error


class PersonFollower(object):
	"""docstring for PersonFollower"""
	def __init__(self):
		# self.CoM, self.distance_away = calculateCenterOfMass()
		pass

	def calculateCenterOfMass(self, ranges):
		ranges = list(ranges)
		view = ranges[300:360] + ranges[0:60]
		indices = []
		for index, reading in enumerate(view):
			if reading > 0 and reading < 1.5:
				indices.append(index)
		try:
			CoM = sum(indices)/len(indices)
		except:
			return (0, 0)

		CoM = int((CoM - 60))
		CoM_distances = []
		for i in range(5):
			if ranges[CoM -i] > 0:
				CoM_distances.append(ranges[CoM - i])
			if ranges[CoM + i] >0:
				CoM_distances.append(ranges[CoM + i])

		try: 
			avg_distance = sum(CoM_distances)/len(CoM_distances)
		except:
			return(CoM, 0)


		return CoM, avg_distance

class ObstacleAvoider(object):
	def __init__(self):
		pass
	
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

	def returnHome(self, odom):
		distance_away = math.sqrt(odom[0]**2 + odom[1]**2)
		angle_away = math.atan(odom[1]/odom[0])
		odom_in_degrees = odom[2]
		if odom[0]>= 0:
			twist_to_home = -self.angle_diff(odom_in_degrees+math.pi,angle_away)
		else:
			twist_to_home = -self.angle_diff(odom_in_degrees,angle_away)


		return (twist_to_home), distance_away

	def seeObstacle(self, ranges):
		ranges = list(ranges)
		pos_half = ranges[0:60]
		neg_half = ranges[300:360]

		total_force = 0

		for i in range(len(pos_half)):
			if pos_half[i] > 0:
				total_force = total_force + (math.cos(i*math.pi/180))/pos_half[i]**2
		print total_force
		for i in range(len(neg_half)):
			if neg_half[i] > 0:
				total_force = total_force - (math.cos((i+270)*math.pi/180))/neg_half[i]**2
		print total_force
		return total_force






			


if __name__ == '__main__':
	run = FiniteStateMachine("ObstacleAvoidance")
	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		run.run()
		r.sleep()
