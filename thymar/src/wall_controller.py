import rospy
import movement_utils as mv
from pid import PID
from geometry_msgs.msg import Twist, Pose, Point
from math import pi, cos, sin


class WallController:
	def __init__(self, proximity_threshold=0.035, wall_safety_distance=2., debug=False):
		self.proximity_min_range = 0.010
		self.proximity_max_range = 0.12
		# Distance between the rear proximity sensors and the reference of the robot
		self.rear_proximity_offset = 0.03
		# Extra distance from wall to cover once the proximity rear sensor stop detecting the wall
		self.distance = wall_safety_distance - self.proximity_max_range - self.rear_proximity_offset

		# --- STATE VARIABLES ---
		self.WALL_REACHED = False
		self.PERPENDICULAR = False
		self.ROTATING = False
		self.ROTATED = False
		self.MAX_RANGE = False
		self.DONE = False

		self.proximity_threshold = proximity_threshold
		self.motion_controller = mv.ToTargetPController(linear_speed=0.20,orientation_speed = 2.5)
		self.debug = debug

		self.angular_vel_pid = PID(Kp=20.,Ki= 0.,Kd = 0.)
		self.velocity = Twist()

		
	def move_closer(self, proximity, position, orientation):
		# Move ahead until the proximity sensors detect an obstacle,
		# then get closer than proximity threshold meters

		if proximity[1] > 0.11 and proximity[2] > 0.11 and proximity[3] > 0.11:
			self.velocity.linear.x = .14
		elif proximity[1] > self.proximity_threshold and proximity[2] > self.proximity_threshold and proximity[3] > self.proximity_threshold:
			self.velocity.linear.x = 0.033
		else:
			self.velocity.linear.x = 0.
			self.WALL_REACHED = True

			self.flat_surface = True
			for i in range(1,4):
				if proximity[i] > 0.119:
					self.flat_surface = False
			print("Obstacle reached. Turning...")

	"""
	If the obstacle is sufficiently wide the proximity sensor are used to rotate the robot to be orthogonal 
	to the obstacle, otherwise no alignement is done (i.e. the robots simply turns by 180 deg with respect
	to the initial direction.
	"""
	def align_perpendicular(self, proximity, position, orientation):
		# Use the frontal proximity sensor to rotate the robot until its is perpendicular to the wall
		max_orientation_speed = 0.75

		if self.flat_surface:
			angular_vel = self.angular_vel_pid.step(proximity[3] - proximity[1], dt=0.1)
			module = min(abs(angular_vel), max_orientation_speed)
			angular_vel *= module / abs(angular_vel)
			
			self.velocity.linear.x = 0.
			self.velocity.angular.z = angular_vel
		else:
			angular_vel = 0.

		if abs(angular_vel) < 0.01:
			self.PERPENDICULAR=True
			self.target_orientation = (orientation+pi)%(2*pi)

	def turn_180(self, proximity, position, orientation):
		done , vel = self.motion_controller.move(position,orientation,
			position,target_orientation=self.target_orientation,
			max_orientation_speed=.75
		)
		self.velocity.linear.x = vel.linear.x
		self.velocity.angular.z = vel.angular.z

		if done:
			self.ROTATED = True
			print("Done.")

	def move_max_range(self, proximity, position, orientation):
		# Move the robot away until the rear proximity sensors reach max range
		
		self.velocity.angular.z = 0.

		if proximity[5] < 0.1199:
			self.velocity.linear.x = 0.1
		else:
			self.velocity.linear.x = 0.

			self.final_target = Point()

			# If self.distance < 0, then the robot is already far enough from the wall
			self.final_target.y = position.y + sin(orientation) * (self.distance if self.distance > 0 else 0.)
			self.final_target.x = position.x + cos(orientation) * (self.distance if self.distance > 0 else 0.)
			self.MAX_RANGE = True

	def move_away(self, proximity, position, orientation):
		# Move away from obstacle until safety distance is reached
		done , vel = self.motion_controller.move(position,orientation,
			self.final_target, max_linear_speed = 0.2
		)
		self.velocity.linear.x = vel.linear.x
		self.velocity.angular.z = vel.angular.z

		if self.debug:
			print(self.velocity)
			print("{0} --> {1}".format(position.x,self.final_target.x))
		if done:
			# print("Done. Final position: ({0},{1})".format(position.x,position.y))
			self.DONE = True

	def reset(self):
		self.WALL_REACHED = False
		self.PERPENDICULAR = False
		self.ROTATING = False
		self.ROTATED = False
		self.MAX_RANGE = False
		self.DONE = False

	def is_obstacle_present(self, proximity):
		return False if proximity[1] > 0.11 and proximity[2] > 0.11 and proximity[3] > 0.11 else True
		
	
	def run(self, proximity, position, orientation):
		
		if not self.WALL_REACHED:
			self.move_closer(proximity, position, orientation)

		if self.WALL_REACHED and not self.PERPENDICULAR:
			self.align_perpendicular(proximity, position, orientation)

		if not self.ROTATED and self.PERPENDICULAR:
			self.turn_180(proximity, position, orientation)
			
		if not self.MAX_RANGE and self.ROTATED:
			self.move_max_range(proximity, position, orientation)

		if not self.DONE and self.MAX_RANGE:
			self.move_away(proximity, position, orientation)

		if self.DONE:
			self.velocity.linear.x = 0.
			self.velocity.angular.z = 0.
			
		return self.velocity





