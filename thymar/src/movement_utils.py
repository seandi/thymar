from enum import Enum
import numpy as np
from math import pow, atan2, sqrt, pi

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point



class Status(Enum):
    EXPLORING_RANDOM = 1
    EXPLORING_SMART = 2
    EXPLORING_COVERAGE = 3
    CHASING_GOAL = 5
    CHASING_TARGET = 6
    RETURNING = 10
    END = 11


class Pose2D:
    def __init__(self, x = None, y = None, theta = None):
        self.x = x
        self.y = y
        self.theta = theta

class Target:
    def __init__(self, pose = Pose2D(), radius = None):
        self.pose = pose
        self.radius = radius


def euclidean_distance(position, target):
	return sqrt(pow((target.x - position.x), 2) +
				pow((position.y - target.y), 2))


def euclidean_distance_tuple(current_pose, target_pose):
	""" Same as `euclidean_distance` but it accept tuples (x,y) instead of pose objects. """
	return sqrt(pow((target_pose[0] - current_pose[0]), 2) +
				pow((target_pose[1] - current_pose[1]), 2))


def to_positive_angle(angle):
	return (angle+2*pi)%(2*pi)




class ToTargetPController:
	def __init__(self, linear_speed, orientation_speed, linear_threshold=0.05, orientation_eps=0.008):
		self.gain1 = linear_speed
		self.gain2 = orientation_speed
		self.linear_eps = linear_threshold
		self.orientation_eps = orientation_eps

	
	def min_angle_diff(self, angle, target_angle):
		delta = target_angle - angle
		if abs(delta) > pi+0.0001:
			module = 2*pi - abs(delta)
			if delta > 0:
				min_delta = -1. * module
			else:
				min_delta = module
		else:
			min_delta=delta
		return min_delta


	def move(self, position, orientation, target, target_orientation=None,
	 max_orientation_speed=None, max_linear_speed=None, custom_distance_tollerance = None):
		velocity = Twist()
		done = True
		distance_current = euclidean_distance(position, target)
		distance_eps = custom_distance_tollerance or self.linear_eps

		if distance_current >= distance_eps:

			vector = (target.x - position.x, target.y - position.y)
			norm = np.linalg.norm([vector[0],vector[1]])
			cos_angle = np.math.acos(vector[0]/norm)
			angle_to_face_target = cos_angle if np.sign(vector[1]) > 0 else -cos_angle

			if distance_current >= distance_eps * 2 and abs(self.min_angle_diff(orientation,angle_to_face_target)) >= self.orientation_eps * 5:
				# -- Turning towards the target
				# print('steering')
				velocity.linear.x = 0.
				velocity.angular.z = self.gain2 * self.min_angle_diff(orientation,angle_to_face_target)
				if max_orientation_speed is not None:
					module = min(abs(velocity.angular.z), max_orientation_speed)
					velocity.angular.z *= module / abs(velocity.angular.z)

				done = False

			else:
				# -- Moving towards the target
				# print('moving')
				velocity.linear.x = self.gain1 #* euclidean_distance(position, target) 
				velocity.angular.z = 0.
				if max_linear_speed is not None:
					velocity.linear.x = min(velocity.linear.x, max_linear_speed)
				done = False

		# -- Turning towards the target orientation
		if done and target_orientation is not None and abs(self.min_angle_diff(orientation,target_orientation)) >= self.orientation_eps:
			# print('final steering')
			velocity.linear.x = 0.
			velocity.angular.z = self.gain2 * self.min_angle_diff(orientation,target_orientation)
			if max_orientation_speed is not None:
				module = min(abs(velocity.angular.z), max_orientation_speed)
				velocity.angular.z *= module / abs(velocity.angular.z)

			done = False

		return done, velocity

		