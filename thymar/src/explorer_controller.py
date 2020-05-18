import rospy
from pid import PID
from geometry_msgs.msg import Twist, Pose, Point
from math import pi
import random

from wall_controller import WallController
import movement_utils as mv

class ExplorerController:
	
	def __init__(self):
		self.obstacle_controller = WallController(proximity_threshold=0.05,
		 wall_safety_distance=.15, debug=False)
		self.motion_controller = mv.ToTargetPController(linear_speed=0.20,orientation_speed = 2.5)

		# ----- STATE VARIABLES -----
		self.INIT = True
		self.EXPLORING = False
		self.OBSTACLE_DETECTED = False
		self.OBSTACLE_AVOIDED = False

		self.velocity = Twist()

		rand_init_dir = random.randint(0, 7)
		# self.init_orientation = pi / 4 * rand_init_dir
		self.init_orientation = 0

		self.velocity.linear.x = 0.13

	def new_rand_orientation(self, orientation, range_min, range_max, n):
		rand_dir = random.randint(0,n)
		step = (range_max-range_min)/(n)
		mid_range  = (range_max-range_min)/2.
		return mv.to_positive_angle(orientation - mid_range + step*rand_dir)

	def explore(self, proximity, position, orientation):
		self.velocity.angular.z = 0

		if self.obstacle_controller.is_obstacle_present(proximity):
			self.EXPLORING = False
			self.OBSTACLE_DETECTED = True

			self.velocity.linear.x = 0.
			return

		self.velocity.linear.x = 0.15

	def run(self, proximity, position, orientation):
		
		# Rotate according to the initial random orientation
		if self.INIT:
			done, vel = self.motion_controller.move(position, orientation,
													position, target_orientation=self.init_orientation,
													max_orientation_speed=.75
													)
			self.velocity.linear.x = vel.linear.x
			self.velocity.angular.z = vel.angular.z

			if done:
				self.INIT = False
				self.EXPLORING = True

		# Move ahead until an obstacle is dettected
		if self.EXPLORING:
			self.explore(proximity, position, orientation)

		# Use wall controller logic to turn away from the obstacle, then chose a new random orientation
		if self.OBSTACLE_DETECTED:
			vel = self.obstacle_controller.run(proximity, position, orientation)
			self.velocity.linear.x = vel.linear.x
			self.velocity.angular.z = vel.angular.z

			if self.obstacle_controller.DONE:
				self.OBSTACLE_AVOIDED = True
				self.OBSTACLE_DETECTED = False
				self.new_orientation = self.new_rand_orientation(orientation=orientation,
				range_min=0., range_max=pi, n=4)

		# Rotate in the new orientation and restart exploration
		if self.OBSTACLE_AVOIDED:
			done, vel = self.motion_controller.move(position, orientation,
													position, target_orientation=None,
													max_orientation_speed=.75
													)
			self.velocity.linear.x = vel.linear.x
			self.velocity.angular.z = vel.angular.z

			if done:
				self.obstacle_controller.reset()
				self.EXPLORING = True
				self.OBSTACLE_DETECTED = False
				self.OBSTACLE_AVOIDED = False

		return self.velocity
