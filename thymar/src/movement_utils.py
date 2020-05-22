from enum import Enum
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from math import pow, atan2, sqrt, pi


class Status(Enum):
    EXPLORING_RANDOM = 1
    EXPLORING_UNKNOWN = 2
    CHASING_GOAL = 3


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

def min_angle_diff(angle, target_angle):
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

def to_positive_angle(angle):
	return (angle+2*pi)%(2*pi) 

def steering_angle(position, target):
	alpha =  to_positive_angle(atan2(target.y - position.y,target.x - position.x))
	return min_angle_diff(0, alpha)

def circular_motion(speed, radius, positive_orientation=True):
	velocity = Twist()
	velocity.linear.y = 0
	velocity.linear.z = 0
	velocity.angular.x = 0
	velocity.angular.y = 0

	velocity.linear.x = speed
	velocity.angular.z = speed/radius
	if not positive_orientation:
		velocity.angular.z = - velocity.angular.z

	return velocity

class ToTargetPController:
	def __init__(self, linear_speed, orientation_speed, linear_threshold=0.05, orientation_eps=0.008):
		self.gain1 = linear_speed
		self.gain2 = orientation_speed
		self.linear_eps = linear_threshold
		self.orientation_eps = orientation_eps

	def move(self, position, orientation, target, target_orientation=None,
	 max_orientation_speed=None, max_linear_speed=None):
		velocity = Twist()
		done = True

		if euclidean_distance(position, target) >= self.linear_eps:
			velocity.linear.x = self.gain1 * euclidean_distance(position, target) 
			if max_linear_speed is not None:
				velocity.linear.x = min(velocity.linear.x, max_linear_speed)
			done = False

		"""
		if abs(steering_angle(position,target)) >= self.linear_eps:
			velocity.angular.z = 3*self.gain1 * steering_angle(position,target)
			done = False
		"""
		
		if target_orientation is not None and abs(min_angle_diff(orientation,target_orientation)) >= self.orientation_eps:
			velocity.linear.x = 0.
			velocity.angular.z = self.gain2 * min_angle_diff(orientation,target_orientation)
			if max_orientation_speed is not None:
				module = min(abs(velocity.angular.z), max_orientation_speed)
				velocity.angular.z *= module / abs(velocity.angular.z)

			done = False

		return done, velocity

		
