from enum import Enum
import numpy as np
from matplotlib import pyplot as plt

import rospy
from geometry_msgs.msg import Twist, Pose, Point

import utils_occupancygrid as grid_utils
import utils_movement as move_utils
from utils_movement import Pose2D, Target



class Status(Enum):
    EXPLORING_RANDOM = 1
    EXPLORING_SMART = 2
    EXPLORING_COVERAGE = 3
    CHASING_GOAL = 5
    CHASING_TARGET = 6
    RETURNING = 10
    END = 11



class ThymarController:
	""" Manages the map exploration until the target is reached, then go back to the initial position. """
	

	def __init__(self, grid_resolution, initital_status, 
				status_after_founding_target = Status.CHASING_TARGET, 
				status_after_reaching_target = Status.EXPLORING_COVERAGE,
				status_after_mapcoverage = Status.RETURNING,
				allow_unknown_traversing = False,
				robot_visibility = 1.1, robot_width = 0.2,
				obstacle_identifier = 100, unknown_identifier = -1):

		self.motion_controller = move_utils.ToTargetPController(linear_speed=0.13, orientation_speed = 2.5)
		
		self.velocity = Twist()
		self.grid_resolution = grid_resolution
		self.robot_width = robot_width
		self.robot_visibility = robot_visibility
		self.obstacle_identifier = obstacle_identifier
		self.unknown_identifier = unknown_identifier

		self.target = Target() # target found in the map
		self.target_found = False
		self.target_chasing = False
		self.target_caught = False
		self.target_distance_tollerance = None

		self.status = initital_status # current status
		self.next_status = None # keeps track of the main task while performing sub-tasks (e.g., tracking path planning while goal chasing)
		self.status_after_founding_target = status_after_founding_target
		self.status_after_reaching_target = status_after_reaching_target
		self.status_after_mapcoverage = status_after_mapcoverage

		self.goal = Pose2D() # goal to reach if self.status == Status.CHASING_GOAL
		self.goal_distance_tollerance = None

		self.stucked_count = 0 # used to count how many times the robot get stucked
		self.planning_count = 0 # used to count how many timesteps are elapsed since the last path planning
		self.planning_path = [] # tracks path to follow
		self.allow_unknown_traversing = allow_unknown_traversing
		self.obstacle_safe_distance = np.ceil(self.robot_width/self.grid_resolution*0.75).astype(int)



	# ------------------- CONTROLLERS


	def explore_random(self, position, orientation, occupancy_grid, steering_step = 5, stucked_max = 8):
		""" Randomly explores the environment avoiding obstacles. """

		current_x = position.x
		current_y = position.y
		current_theta = orientation

		obs_present, obs_dist, obs_odom = grid_utils.has_facing_obstacle(occupancy_grid, self.grid_resolution, 
																		current_x, current_y, current_theta,
																		robot_visibility=self.robot_visibility, robot_width=self.robot_width)

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
				_, dist_left, _ = grid_utils.has_facing_obstacle(occupancy_grid, self.grid_resolution, 
									current_x, current_y, current_theta + np.deg2rad(i), 
									robot_visibility=2*self.robot_visibility, robot_width=self.robot_width)
				_, dist_right, _ = grid_utils.has_facing_obstacle(occupancy_grid, self.grid_resolution, 
									current_x, current_y, current_theta - np.deg2rad(i), 
									robot_visibility=2*self.robot_visibility, robot_width=self.robot_width)
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
				obs_present, obs_dist, obs_odom = grid_utils.has_facing_obstacle(occupancy_grid, self.grid_resolution, 
																				current_x, current_y, current_theta, 
																				robot_visibility=2*self.robot_visibility,
																				robot_width=self.robot_width)

			# stops the robot and set goal for reaching the computed theta
			self.next_status = self.status
			self.status = Status.CHASING_GOAL
			self.goal = Pose2D(current_x, current_y, current_theta)
			self.goal_distance_tollerance = None
			self.velocity = Twist()
			rospy.loginfo('Steering for reaching theta {:.2f}...'.format(current_theta))



	def explore_covering(self, position, orientation, occupancy_grid, 
						recomputation = 10, skip_poses = 3, nopath_status = Status.END):
		""" Explores the environment while always managing to reach the nearest unknown position in the map. """

		# path planning is only recomputed every `recomputation` timesteps or the path is too short
		min_path_length = np.ceil(self.robot_width / self.grid_resolution/2).astype(int)
		if len(self.planning_path) < min_path_length or self.planning_count % recomputation == 0: 
			rospy.loginfo('Recomputing path for reaching unknown areas...')
			sx = position.x	
			sy = position.y
			self.planning_path = grid_utils.path_planning(occupancy_grid, self.grid_resolution,
															self.allow_unknown_traversing,
															start_odom = (sx, sy), 
															goal_odom = None, goal_identifier = -1,
															obstacle_identifier = self.obstacle_identifier, 
															unknown_identifier = self.unknown_identifier,
															collision_expansion = self.obstacle_safe_distance)
			
			if self.planning_path == None or len(self.planning_path) == 0:
				self.velocity = Twist() # stops the robot
				self.status = nopath_status
				rospy.loginfo('Cannot find a feasible path for unknown areas!')
				rospy.loginfo('Status changed to ' + str(self.status))
				return

			gx = self.planning_path[-1][0]
			gy = self.planning_path[-1][1]

			rospy.loginfo('Computed path from from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f}) of total lenght = {}'
							.format(sx, sy, gx, gy, len(self.planning_path)))

		# for kinematics reasons, skips some of the planned poses and retrieve the next one to be reached
		for _ in range(skip_poses + 1):
			if len(self.planning_path) > 1: # at least 2
				next_pose = self.planning_path.pop(0)
			else:
				break

		next_pose = self.planning_path.pop(0)
		next_area = occupancy_grid[grid_utils.odom_to_grid(next_pose, self.grid_resolution)[::-1]]
		self.velocity = Twist() # stops the robot

		if next_area == -1 or next_area == 100:
			# it is possible that new explored areas are unknown or obstacles, so better not to reach them
			self.planning_count = 0
			rospy.loginfo('Early path recomputation because unknown area or obstacle has been seen on the path')
		else:
			self.planning_count += 1
			# set goal for reaching the pose while setting next status the same as now
			self.goal = Pose2D(next_pose[0], next_pose[1], None)
			self.goal_distance_tollerance = None
			self.next_status = self.status
			self.status = Status.CHASING_GOAL
			rospy.loginfo('Current goal is ({:.2f}, {:.2f})'.format(next_pose[0], next_pose[1]))

	
	def chase_planning(self, position, orientation, occupancy_grid, 
						goal, goal_orientation, status_after_finish, 
						recomputation = 6, skip_poses = 3):
		""" 
			Chase a goal pose by using path planning. 
			Path is recomputed after a number `recomputation` of intermediate poses reached;
			if it is <= 0, the path is never recomputed until the goal is reached. 
		"""

		grid_with_target = np.copy(occupancy_grid)
		
		# if chasing target, do not consider it as an obstacle
		if self.target_found and self.target_chasing:
			tx, ty = grid_utils.odom_to_grid((self.target.pose.x, self.target.pose.y), self.grid_resolution)
			w = np.ceil(self.target_distance_tollerance/self.grid_resolution).astype(int) + 1
			grid_with_target[ty-w:ty+w, tx-w:tx+w] = self.obstacle_identifier - 5 # different from `obstacle_identifier`


		if len(self.planning_path) == 0 or self.planning_count % recomputation == 0: 
			rospy.loginfo('Recomputing path for chasing ({:.2f}, {:.2f}) ...'.format(goal.x, goal.y))
			sx = position.x	
			sy = position.y
			self.planning_path = grid_utils.path_planning(grid_with_target, self.grid_resolution,
															self.allow_unknown_traversing,
															start_odom = (sx, sy),  
															goal_odom = (goal.x, goal.y), goal_identifier = None,
															obstacle_identifier = self.obstacle_identifier, 
															unknown_identifier = self.unknown_identifier,
															collision_expansion = self.obstacle_safe_distance)
			
			if len(self.planning_path) == 0:
				self.velocity = Twist() # stops the robot
				self.status = Status.END
				rospy.loginfo('Cannot find a feasible path for chasing ({:.2f}, {:.2f}) ...'.format(goal.x, goal.y))
				rospy.loginfo('Status changed to ' + str(self.status))
				return

			gx = self.planning_path[-1][0]
			gy = self.planning_path[-1][1]

			rospy.loginfo('Computed path from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f}) of total lenght = {}'
							.format(sx, sy, gx, gy, len(self.planning_path)))

		# for kinematics reasons, skips some of the planned poses and retrieve the next one to be reached
		for _ in range(skip_poses):
			if len(self.planning_path) > 1:
				next_pose = self.planning_path.pop(0)
			else:
				break

		next_pose = self.planning_path.pop(0)
		next_area = grid_with_target[grid_utils.odom_to_grid(next_pose, self.grid_resolution)[::-1]]
		finished = True if len(self.planning_path) == 0 else False
		self.velocity = Twist() # stops the robot

		if next_area == self.unknown_identifier or next_area == self.obstacle_identifier:			
			# it is possible that new explored areas are unknown or obstacles, so better not to reach them
			self.planning_count = 0
			rospy.loginfo('Early path recomputation because unknown area or obstacle has been seen on the path')
		else:
			self.planning_count += 1
			# set goal for reaching the pose while setting next status as specified, if the final goal has been reached
			self.goal = Pose2D(next_pose[0], next_pose[1], goal_orientation if finished else None)
			self.next_status = status_after_finish if finished else self.status # after current goal has been reached, returns in the previous state
			self.status = Status.CHASING_GOAL
			rospy.loginfo('Current goal is ({:.2f}, {:.2f})'.format(next_pose[0], next_pose[1]))



	def chase_straight(self, position, orientation, goal, goal_orientation):

		done, vel = self.motion_controller.move(position, orientation,
												goal, target_orientation=goal_orientation,
												max_linear_speed=.13, max_orientation_speed=.5)

		self.velocity.linear.x = vel.linear.x
		self.velocity.angular.z = vel.angular.z

		if done:
			rospy.loginfo('Intermediate goal reached')
			self.status = self.next_status

			if self.status == Status.END:
				rospy.loginfo('END REACHED!')



	# ------------------- STATES HANDLER


	def run(self, position, orientation, occupancy_grid):
		""" Returns proper velocities accordingly to the current status. """
		
		if self.target_found and not self.target_chasing and not self.target_caught:
			rospy.loginfo('TARGET IDENTIFIED!')
			self.velocity = Twist()
			self.planning_count = 0
			self.target_chasing = True
			self.target_distance_tollerance = self.target.radius * 2 + self.robot_width
			self.status = self.status_after_founding_target
			self.next_status = None
			rospy.loginfo('Status changed to ' + str(self.status))

		elif self.target_found and self.target_chasing and move_utils.euclidean_distance(position, self.target.pose) < self.target_distance_tollerance:
			rospy.loginfo('TARGET REACHED!')
			self.velocity = Twist()
			self.planning_count = 0
			self.target_chasing = False
			self.target_caught = True
			self.status = self.status_after_reaching_target
			self.next_status = None
			rospy.loginfo('Status changed to ' + str(self.status))

		elif self.status == Status.EXPLORING_RANDOM:
			self.explore_random(position, orientation, occupancy_grid)

		elif self.status == Status.EXPLORING_SMART:
			self.explore_covering(position, orientation, occupancy_grid)

		elif self.status == Status.EXPLORING_COVERAGE:
			self.explore_covering(position, orientation, occupancy_grid,
								nopath_status = self.status_after_mapcoverage)

		elif self.status == Status.CHASING_GOAL:
			self.chase_straight(position, orientation, self.goal, self.goal.theta)

		elif self.status == Status.CHASING_TARGET:
			self.chase_planning(position, orientation, occupancy_grid, 
								self.target.pose, goal_orientation = None, 
								recomputation = 2 if self.allow_unknown_traversing else 100,
								status_after_finish = self.status_after_reaching_target)

		elif self.status == Status.RETURNING:
			self.chase_planning(position, orientation, occupancy_grid, 
								Pose2D(0, 0), goal_orientation = 0, 
								recomputation = 2 if self.allow_unknown_traversing else 100,
								status_after_finish = Status.END)

		elif self.status == Status.END:
			rospy.loginfo('Status END')

		return self.velocity
