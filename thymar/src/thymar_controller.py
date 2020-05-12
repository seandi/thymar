import rospy
from geometry_msgs.msg import Twist, Pose, Point
from math import pi
import random


class ThymarController:
	
	def __init__(self, grid_resolution):
		self.velocity = Twist()
		self.velocity.linear.x = 0.
		self.velocity.linear.y = 0.
		self.velocity.linear.z = 0.
		self.velocity.angular.x = 0.
		self.velocity.angular.y = 0.
		self.velocity.angular.z = 0.

		self.grid_resolution = grid_resolution
		self.target = None

		# ROBOT STATES
		self.EXPLORE = True
		self.REACH_TARGET = False

	def setTarget(self, target):
		self.target = target
		self.EXPLORE = False
		self.REACH_TARGET = True

	
	def run(self,position, orientation, occupancy_grid):
		
		# EXPLORE 
		if self.EXPLORE:
			pass

		# PATH PLANNING
		if self.REACH_TARGET:
			pass

		return self.velocity
