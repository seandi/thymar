import numpy as np
from matplotlib import pyplot as plt

import rospy
from geometry_msgs.msg import Twist, Pose, Point

import movement_utils as mv
from movement_utils import Pose2D, Target, Status



class ThymarController:
	""" Manages the map exploration until the target is reached, then go back to the initial position. """
	

	def __init__(self, grid_resolution,
				robot_visibility = 1.1, robot_width = 0.2, default_status = Status.EXPLORING_RANDOM):

		self.velocity = Twist()
		self.grid_resolution = grid_resolution
		self.robot_width = robot_width
		self.robot_visibility = robot_visibility

		self.default_status = default_status
		self.status = default_status
		self.motion_controller = mv.ToTargetPController(linear_speed=0.15, orientation_speed = 2.5)
		
		self.target = Target() # target found in the map
		self.goal = Pose2D() # goal to reach if self.status == Status.CHASING_GOAL
		self.stucked_count = 0 # used to count how many times the robot get stucked



	def odom_to_grid(self, odom):
		""" Transforms Odom coordinates to OccupancyGrid coordinates. """

		x_grid = round(odom[0] / self.grid_resolution) + 200
		y_grid = round(odom[1] / self.grid_resolution) + 200
		return int(x_grid), int(y_grid)


	def grid_to_odom(self, grid):
		""" Transforms OccupancyGrid coordinates to Odom coordinates. """

		x_odom = (grid[0] - 200) * self.grid_resolution
		y_odom = (grid[1] - 200) * self.grid_resolution
		return x_odom, y_odom
	


	def pose_ahead(self, x, y, theta, meters_ahead):
		""" Computes a pose that is `meters_ahead` meters ahead wrt the current pose. """

		ahead_x = meters_ahead * np.cos(theta) + x
		ahead_y = meters_ahead * np.sin(theta) + y
		return ahead_x, ahead_y



	def has_facing_obstacle(self, grid, x, y, theta, robot_visibility = None):
		""" Given the current pose, check if the robot is facing an obstacle. """

		robot_visibility = min(robot_visibility, 2.5) or self.robot_visibility
		step = self.grid_resolution

		obstacle_facing = False
		obstacle_distance = None
		obstacle_odom = None

		adjacent_cells = np.ceil(self.robot_width/step/2).astype(int)
		theta_orthogonal = theta + np.pi/2

		for i in range(int(robot_visibility//step)):

			distance = step * i
			ahead_odom = self.pose_ahead(x, y, theta, distance)
			ahead_grid = self.odom_to_grid(ahead_odom)

			if grid[ahead_grid[::-1]] == 100:
				obstacle_facing = True
			else:
				ahead_grid_near = [] # used for checking cells that would be hit by robot width
				for j in range(1, adjacent_cells + 1):
					l = self.odom_to_grid(self.pose_ahead(ahead_odom[0], ahead_odom[1], theta_orthogonal, j *  self.grid_resolution))
					r = self.odom_to_grid(self.pose_ahead(ahead_odom[0], ahead_odom[1], theta_orthogonal, j * -self.grid_resolution))
					ahead_grid_near.append(l)
					ahead_grid_near.append(r)
			
			ahead_grid_near = list(set(ahead_grid_near))
			# print('grid:', ahead_grid, '\t adj:', ahead_grid_list, '\t odom:', ahead_odom)

			for j in range(len(ahead_grid_near)):
				if grid[ahead_grid_near[j][::-1]] == 100: 
					obstacle_facing = True
					break

			if obstacle_facing:
				obstacle_distance = distance
				obstacle_odom = ahead_odom
				break
		
		return obstacle_facing, obstacle_distance, obstacle_odom



	def explore_random(self, position, orientation, occupancy_grid, steering_step = 5, stucked_max = 8):
		""" Randomly explores the environment avoiding obstacles. """

		current_x = position.x
		current_y = position.y
		current_theta = orientation

		obs_present, obs_dist, obs_odom = self.has_facing_obstacle(occupancy_grid, current_x, current_y, current_theta)

		if(not obs_present): # movement ahead
			self.stucked_count = 0
			self.velocity.linear.x = 0.15
			self.velocity.angular.z = 0.

		else: # look for a free path by turning around
			rospy.loginfo('Facing obstacle at ({:.2f}, {:.2f}), {:.2f} meters far (theta {:.2f})'.format(obs_odom[0], obs_odom[1], obs_dist, current_theta))

			self.stucked_count += 1
			if self.stucked_count > stucked_max: # prevent the robot to be stucked in trying the same orientations over and over
				current_theta += np.deg2rad(np.random.uniform(-130, 130))

			# evaluating left and right side of the robot to choose the turning direction
			left_distances = []
			right_distances = []
			for i in range(1, 45, 5):
				# please note that here the robot has double the normal visibility range
				_, dist_left, _ = self.has_facing_obstacle(occupancy_grid, current_x, current_y, current_theta + np.deg2rad(i), robot_visibility=2*self.robot_visibility)
				_, dist_right, _ = self.has_facing_obstacle(occupancy_grid, current_x, current_y, current_theta - np.deg2rad(i), robot_visibility=2*self.robot_visibility)
				left_distances.append(dist_left or 4)
				right_distances.append(dist_right or 4)

			# once both sides are evaluated, the robot tries to turn the preferred way for finding a free path
			deg_delta = steering_step if np.mean(left_distances) > np.mean(right_distances) else -steering_step # positive or negative steering based on more free way
			deg_delta *= 1 / self.robot_visibility # multiplier for adjusting steering step on visibility distance (default is 1 meters visibility = `steering_step` degrees) 
			previous_obs_dist = 0
			steering_count = 0
			while obs_present and obs_dist * 1.5 >= previous_obs_dist: # allows to take still-blocked ways that are better than before
				steering_count += 1
				previous_obs_dist = obs_dist
				if steering_count % (360/deg_delta) == 0: # every time a searching round is complete...
					deg_delta /= 2 # ... steering angle decays 
				current_theta += np.deg2rad(deg_delta) # step-by-step research
				# please note that here the robot has double the normal visibility range
				obs_present, obs_dist, obs_odom = self.has_facing_obstacle(occupancy_grid, current_x, current_y, current_theta, robot_visibility=2*self.robot_visibility)

			# stops the robot and set goal for reaching the computed theta
			self.goal = Pose2D(current_x, current_y, current_theta)
			self.status = Status.CHASING_GOAL
			self.velocity.linear.x = 0.
			self.velocity.angular.z = 0.
			rospy.loginfo('Steering for reaching theta {:.2f}...'.format(current_theta))


	
	def run(self, position, orientation, proximity, occupancy_grid):
		""" Returns proper velocities accordingly to the current status. """

		if self.status == Status.CHASING_GOAL:
			done, vel = self.motion_controller.move(position, orientation,
													self.goal, target_orientation=self.goal.theta,
													max_linear_speed=.15, max_orientation_speed=.75)
			self.velocity.linear.x = vel.linear.x
			self.velocity.angular.z = vel.angular.z

			if done:
				self.status = self.default_status
		

		elif self.status == Status.EXPLORING_RANDOM:
			self.explore_random(position, orientation, occupancy_grid)


		elif self.status == Status.EXPLORING_UNKNOWN:
			pass


		return self.velocity
