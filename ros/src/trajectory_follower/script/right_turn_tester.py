#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import geometry_msgs.msg
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
from autogo_msgs.msg import Trajectory, TrajectoryPoint, PathPoint, localization_status


class control_tester:
	def __init__(self, rate):
		self.rate = rospy.Rate(rate)
		self.listener = tf.TransformListener()
		self.position = []
		self.quaternion = []
		self.path = Trajectory()
		self.x = []
		self.y = []
		self.pub1_ = rospy.Publisher(
			"/autogo/planning/trajectory", Trajectory, queue_size=10)
		self.sub = rospy.Subscriber(
			"/autogo/localization/localization_status", localization_status, self.callback)
		self.status = False
		self.score = 0.0
		self.waypoints_marker = Marker()
		self.pub2_ = rospy.Publisher(
			"/autogo/fake_global_trajectory", Marker, queue_size=10)

	def callback(self, data):
		self.status = data.status
		self.score = data.score

	def tfListener(self):
		while not rospy.is_shutdown() and not self.position and not self.quaternion:
			if self.status and self.score > 0.6:
				try:
					self.position, self.quaternion = self.listener.lookupTransform(
						'/map', '/base_link', rospy.Time(0))
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					rospy.logwarn('localization failed!')
			else:
				rospy.logwarn("localization status not good!")
			self.rate.sleep()
		rospy.loginfo("localizaton get!")
		print self.position, self.quaternion

	def tfListenserSpin(self):
		try:
			self.position, self.quaternion = self.listener.lookupTransform(
				"/map", '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException):
			rospy.logwarn('localization failed!')

	def trajectory_generator(self):
		euler = tf.transformations.euler_from_quaternion(self.quaternion)
		yaw = euler[2]
		origin_x = self.position[0]
		origin_y = self.position[1]
		point = TrajectoryPoint()
		point.path_point.point.x = origin_x
		point.path_point.point.y = origin_y
		point.path_point.theta = yaw
		point.v = 1.5
		for i in range(1, 100, 1):
			point_ = TrajectoryPoint()
			point.path_point.point.x += 10.0 / 100.0 * math.cos(yaw)
			point.path_point.point.y += 10.0 / 100.0 * math.sin(yaw)
			point_.path_point.point.x = point.path_point.point.x
			point_.path_point.point.y = point.path_point.point.y
			point_.path_point.theta = point.path_point.theta
			point_.v = 1.5
			self.path.trajectory_points.append(point_)
			self.x.append(point.path_point.point.x)
			self.y.append(point.path_point.point.y)
		for i in range(1, 100, 1):
			point.path_point.theta -= math.pi / 2.0 / 100.0
			point.path_point.point.x += 4.0 * \
				math.sqrt(2.0) / 100.0 * math.cos(point.path_point.theta)
			point.path_point.point.y += 4.0 * \
				math.sqrt(2.0) / 100.0 * math.sin(point.path_point.theta)
			point_ = TrajectoryPoint()
			point_.path_point.point.x = point.path_point.point.x
			point_.path_point.point.y = point.path_point.point.y
			point_.path_point.theta = point.path_point.theta
			point_.v = 1.5
			point_.path_point.kappa = 0.1
			self.path.trajectory_points.append(point_)
			self.x.append(point.path_point.point.x)
			self.y.append(point.path_point.point.y)
		for i in range(1, 100, 1):
			point.path_point.point.x += 10.0 / 100.0 * \
				math.cos(point.path_point.theta)
			point.path_point.point.y += 10.0 / 100.0 * \
				math.sin(point.path_point.theta)
			point_ = TrajectoryPoint()
			point_.path_point.point.x = point.path_point.point.x
			point_.path_point.point.y = point.path_point.point.y
			point_.path_point.theta = point.path_point.theta
			point_.v = 1.5
			self.path.trajectory_points.append(point_)
			self.x.append(point.path_point.point.x)
			self.y.append(point.path_point.point.y)

	def visualization(self, x, y):
		plt.plot(x, y, 'r^')
		plt.show()

	def visualizationSpin(self, x, y):
		plt.ion()
		plt.plot(x, y, 'g*')

	def publisherSpin(self):
		if self.index + 20 <= len(self.path.trajectory_points):
			trajectory_pub = self.path.trajectory_points[self.index:self.index+20]
		else:
			trajectory_pub = self.path.trajectory_points[self.index:]
		trajectory = Trajectory()
		trajectory.trajectory_points = trajectory_pub
		trajectory.type = 1
		self.pub1_.publish(trajectory)
		self.pub2_.publish(self.waypoints_marker)

	def FindNearestPointByPosition(self, x, y):
		dist_min = float("inf")
		self.index = 0
		for i, point in enumerate(self.path.trajectory_points):
			dx = point.path_point.point.x - x
			dy = point.path_point.point.y - y
			dist = math.hypot(dx, dy)
			if dist < dist_min:
				dist_min = dist
				self.index = i

	def run(self):
		while not rospy.is_shutdown():
			self.tfListenserSpin()
			self.FindNearestPointByPosition(self.position[0], self.position[1])
			self.publisherSpin()
#			self.visualizationSpin(self.position[0], self.position[1])
			self.rate.sleep()

	def PointMarker(self):
		self.waypoints_marker = Marker()
		self.waypoints_marker.header.frame_id = "map"
		self.waypoints_marker.header.stamp = rospy.Time.now()
		self.waypoints_marker.ns = "waypoints"
		self.waypoints_marker.id = 0
		self.waypoints_marker.type = Marker.CUBE_LIST
		self.waypoints_marker.action = Marker.ADD
		self.waypoints_marker.scale.x = 0.05
		self.waypoints_marker.scale.y = 0.05
		self.waypoints_marker.color.g = 0.6
		self.waypoints_marker.color.r = 1.0
		self.waypoints_marker.color.b = 0.6
		self.waypoints_marker.color.a = 1.0
		self.waypoints_marker.frame_locked = True
		self.waypoints_marker.pose.orientation.w = 1.0
		for point in self.path.trajectory_points:
			self.waypoints_marker.points.append(point.path_point.point)


if __name__ == "__main__":
	rospy.init_node('control_tester', anonymous=True)
	ct = control_tester(10)
	ct.tfListener()
	ct.trajectory_generator()
	# ct.visualization(ct.x, ct.y)
	ct.PointMarker()
	ct.run()
