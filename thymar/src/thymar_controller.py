import numpy as np
from matplotlib import pyplot as plt

import rospy
from geometry_msgs.msg import Twist, Pose, Point

import movement_utils as mv
from movement_utils import Pose2D, Target, Status



class GraphNode:
	def __init__(self, pose, g_score, prev_node_pose, goal_pose):
		self.pose = pose
		self.g_score = g_score # actual cost
		self.h_score = 0 if goal_pose == None else mv.euclidean_distance_tuple(self.pose, goal_pose) # heuristic cost
		self.prev_node_pose = prev_node_pose

	def f_score(self):
		return self.g_score + self.h_score # priority cost

	def __str__(self):
		return str(self.pose) + ", g:" + str(self.g_score) + ", f:" + "{:.3f}".format(self.f_score()) + ", prev:" + str(self.prev_node_pose)



class ThymarController:
	""" Manages the map exploration until the target is reached, then go back to the initial position. """
	

	def __init__(self, grid_resolution, default_status,
				robot_visibility = 1.1, robot_width = 0.2):

		self.velocity = Twist()
		self.grid_resolution = grid_resolution
		self.robot_width = robot_width
		self.robot_visibility = robot_visibility

		self.default_status = default_status
		self.status = default_status
		self.next_status = None
		self.motion_controller = mv.ToTargetPController(linear_speed=0.15, orientation_speed = 2.5)
		
		self.target = Target() # target found in the map
		self.goal = Pose2D() # goal to reach if self.status == Status.CHASING_GOAL

		self.stucked_count = 0 # used to count how many times the robot get stucked
		self.planning_count = 0 # used to count how many timesteps are elapsed since the last path planning
		self.planning_path = [] # tracks path to follow



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


	# ------------------- EXPLORE COVERING UTILS


	def is_valid_neighbour(self, grid, gpose, obstacle_identifier = 100):
		# When reading this code, please remember that while pose is expressed as (X,Y),
		# the matrix indexing for `grid` is instead to be done as (Y,X).
		# Moreover, the Y is flipped in the map (see maps images), then for moving the
		# robot UP we actually need to increment Y (row_index + 1)
		# instead of decreasing it (such as it would happen in a normal matrix).

		# Due to the robot width, cells next to the obstacles must be ignored for robot traversing
		collision = np.ceil(self.robot_width/self.grid_resolution/2).astype(int)
		
		cells = set([gpose])
		for _ in range(collision): # explore all adjacent cells for a maximum of `collision` layers
			tmp =  set([])
			for _, tup in enumerate(cells):
				x = tup[0]
				y = tup[1]
				tmp.add((x - 1, y)) # left
				tmp.add((x + 1, y)) # right
				tmp.add((x, y - 1)) # down
				tmp.add((x, y + 1)) # up
				tmp.add((x - 1, y - 1)) # diagonal left-down
				tmp.add((x - 1, y + 1)) # diagonal left-up
				tmp.add((x + 1, y - 1)) # diagonal right-down
				tmp.add((x + 1, y + 1)) # diagonal right-up

			for _, tup in enumerate(tmp): # checking cells for validity
				if grid[tup[1], tup[0]] == obstacle_identifier:
					return False

			cells = cells.union(tmp)

		return True


	def get_neighbourhood(self, grid, grid_pose):
		x = grid_pose[0] # x=col
		y = grid_pose[1] # y=row
		neighbourhood = []
		neighbourhood.append((x - 1, y)) # left
		neighbourhood.append((x + 1, y)) # right
		neighbourhood.append((x, y - 1)) # down
		neighbourhood.append((x, y + 1)) # up
		neighbourhood.append((x - 1, y - 1)) # diagonal left-down
		neighbourhood.append((x - 1, y + 1)) # diagonal left-up
		neighbourhood.append((x + 1, y - 1)) # diagonal right-down
		neighbourhood.append((x + 1, y + 1)) # diagonal right-up
		return neighbourhood


	def get_complete_path(self, nodes_set, node):
		path = [node.pose]
		while node.prev_node_pose != None:
			path.append(node.prev_node_pose)
			node = nodes_set[node.prev_node_pose]
		return path[::-1]


	def path_planning(self, grid, start_odom, goal_odom, goal_identifier = None):
		""" Applies A* or Dijkstra depending on the fact that `goal_odom` is specified or not """

		start_grid = self.odom_to_grid(start_odom)
		goal_grid = None if goal_odom == None else self.odom_to_grid(goal_odom)
		
		if goal_grid == None and goal_identifier == None: 
			raise Exception('You must have either a goal pose or color.')

		open_set = dict() # nodes to explore
		closed_set = dict() # already explored nodes
		
		start_node = GraphNode(start_grid, 0, None, goal_grid)
		open_set[start_node.pose] = start_node


		while len(open_set) > 0:

			# get the node with minimum f_score
			chosen_idx = min(open_set, key = lambda dict_idx: open_set[dict_idx].f_score()) 
			current_node = open_set[chosen_idx]
			
			success = True if goal_grid == None or current_node.pose == goal_grid else False
			success = True if success and (goal_identifier == None or grid[current_node.pose[::-1]] == goal_identifier) else False

			if success: 
				return [self.grid_to_odom(p) for p in self.get_complete_path(closed_set, current_node)] # success: goal reached

			del open_set[chosen_idx]
			closed_set[chosen_idx] = current_node

			neighbourhood = self.get_neighbourhood(grid, current_node.pose) # list of neighbours poses (tuple)
			
			for i, neighbour in enumerate(neighbourhood):
			
				if neighbour in closed_set:
					continue # already explored

				if not self.is_valid_neighbour(grid, neighbour):
					continue

				g_score = current_node.g_score + mv.euclidean_distance_tuple(current_node.pose, neighbour)

				if neighbour not in open_set: # new node
					open_set[neighbour] = GraphNode(neighbour, g_score, current_node.pose, goal_grid)
				
				elif open_set[neighbour].g_score > g_score: # best path for reaching the node `neighbour` 
						open_set[neighbour].g_score = g_score
						open_set[neighbour].prev_node_pose = current_node.pose

		return None # failure: open_set is empty but goal was never reached


	# ------------------- CONTROLLERS


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
			self.next_status = Status.EXPLORING_RANDOM
			self.velocity = Twist()
			rospy.loginfo('Steering for reaching theta {:.2f}...'.format(current_theta))


	def explore_covering(self, position, orientation, occupancy_grid, recomputation = 5, skip_poses = 3):
		""" Explores the environment while always managing to reach the nearest undiscovered position in the map. """

		# path planning is only recomputed every `recomputation` timesteps
		if len(self.planning_path) == 0 or (recomputation != -1 and self.planning_count % recomputation == 0): 
			rospy.loginfo('Recomputing path for reaching unknown areas...')
			sx = position.x	
			sy = position.y
			self.planning_path = self.path_planning(occupancy_grid, (sx, sy), None, -1)
			
			if len(self.planning_path) == 0:
				rospy.loginfo('Cannot find a feasible path for unknown areas')
				self.status = Status.END
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
		self.planning_count += 1

		# stops the robot and set goal for reaching the pose
		self.status = Status.CHASING_GOAL
		self.next_status = Status.EXPLORING_COVERAGE
		self.goal = Pose2D(next_pose[0], next_pose[1], None)
		self.velocity = Twist()
		rospy.loginfo('Current goal is ({:.2f}, {:.2f})'.format(next_pose[0], next_pose[1]))

	
	def chase_planning(self, position, orientation, grid, goal, goal_orientation, 
						next_status, tollerance = None, recomputation = 5, skip_poses = 3):
		
		# path planning is only recomputed every `recomputation` timesteps
		if len(self.planning_path) == 0 or (recomputation != -1 and self.planning_count % recomputation == 0): 
			rospy.loginfo('Recomputing path for chasing ({:.2f}, {:.2f}) ...'.format(goal.x, goal.y))
			sx = position.x	
			sy = position.y
			gx = goal.x
			gy = goal.y
			self.planning_path = self.path_planning(grid, (sx, sy), (gx, gy), None)
			
			if len(self.planning_path) == 0:
				rospy.loginfo('Cannot find a feasible path for chasing ({:.2f}, {:.2f}) ...'.format(goal.x, goal.y))
				self.status = Status.END
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
		self.planning_count += 1

		# stops the robot and set goal for reaching the pose
		self.status = Status.CHASING_GOAL
		self.next_status = next_status
		self.goal = Pose2D(next_pose[0], next_pose[1], None)
		self.velocity = Twist()
		rospy.loginfo('Current goal is ({:.2f}, {:.2f})'.format(next_pose[0], next_pose[1]))


	def chase_simple(self, position, orientation, goal, goal_orientation, tollerance = None):
		done, vel = self.motion_controller.move(position, orientation,
												goal, target_orientation=goal_orientation,
												max_linear_speed=.15, max_orientation_speed=.5,
												custom_distance_tollerance=tollerance)
		self.velocity.linear.x = vel.linear.x
		self.velocity.angular.z = vel.angular.z

		if done:
			rospy.loginfo('Intermediate goal reached')
			self.status = self.next_status



	def run(self, position, orientation, proximity, occupancy_grid):
		""" Returns proper velocities accordingly to the current status. """

		if self.target.pose.x != None and self.target.pose.y != None:
			self.chase_planning(position, orientation, occupancy_grid, self.target.pose, None, Status.RETURNING, tollerance=0.30)

		elif self.status == Status.CHASING_GOAL:
			self.chase_simple(position, orientation, self.goal, self.goal.theta)

		elif self.status == Status.EXPLORING_RANDOM:
			self.explore_random(position, orientation, occupancy_grid)

		elif self.status == Status.EXPLORING_COVERAGE:
			self.explore_covering(position, orientation, occupancy_grid)

		elif self.status == Status.RETURNING:
			self.chase_planning(position, orientation, occupancy_grid, Pose2D(0, 0), 0, Status.END)

		return self.velocity
