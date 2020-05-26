import numpy as np
from matplotlib import pyplot as plt

import utils_movement as move_utils
from utils_movement import Pose2D, Target


# ------------------- BASIC UTILS


def odom_to_grid(odom, grid_resolution):
    """ Transforms Odom coordinates to OccupancyGrid coordinates. """

    x_grid = round(odom[0] / grid_resolution) + 200
    y_grid = round(odom[1] / grid_resolution) + 200
    return int(x_grid), int(y_grid)


def grid_to_odom(grid, grid_resolution):
    """ Transforms OccupancyGrid coordinates to Odom coordinates. """

    x_odom = (grid[0] - 200) * grid_resolution
    y_odom = (grid[1] - 200) * grid_resolution
    return x_odom, y_odom



def pose_ahead(x, y, theta, meters_ahead):
    """ Computes a pose that is `meters_ahead` meters ahead wrt the current pose. """

    ahead_x = meters_ahead * np.cos(theta) + x
    ahead_y = meters_ahead * np.sin(theta) + y
    return ahead_x, ahead_y



def has_facing_obstacle(grid, grid_resolution, x, y, theta, robot_visibility, robot_width):
    """ Given the current pose, check if the robot is facing an obstacle. """

    robot_visibility = min(robot_visibility, 2.5)
    step = grid_resolution

    obstacle_facing = False
    obstacle_distance = None
    obstacle_odom = None

    adjacent_cells = np.ceil(robot_width/step/2).astype(int)
    theta_orthogonal = theta + np.pi/2

    for i in range(int(robot_visibility//step)):

        distance = step * i
        ahead_odom = pose_ahead(x, y, theta, distance)
        ahead_grid = odom_to_grid(ahead_odom, grid_resolution)

        if grid[ahead_grid[::-1]] == 100:
            obstacle_facing = True
        else:
            ahead_grid_near = [] # used for checking cells that would be hit by robot width
            for j in range(1, adjacent_cells + 1):
                l = odom_to_grid(pose_ahead(ahead_odom[0], ahead_odom[1], theta_orthogonal, j *  grid_resolution), grid_resolution)
                r = odom_to_grid(pose_ahead(ahead_odom[0], ahead_odom[1], theta_orthogonal, j * -grid_resolution), grid_resolution)
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




# ------------------- PATH PLANNING UTILS


class GraphNode:
	def __init__(self, pose, g_score, prev_node_pose, goal_pose):
		self.pose = pose
		self.g_score = g_score # actual cost
		self.h_score = 0 if goal_pose == None else move_utils.euclidean_distance_tuple(self.pose, goal_pose) # heuristic cost
		self.prev_node_pose = prev_node_pose

	def f_score(self):
		return self.g_score + self.h_score # priority cost

	def __str__(self):
		return str(self.pose) + ", g:" + str(self.g_score) + ", f:" + "{:.3f}".format(self.f_score()) + ", prev:" + str(self.prev_node_pose)



def is_valid_neighbour(grid, gpose, obstacle_identifier = 100, collision_expansion = 3):
    # When reading this code, please remember that while pose is expressed as (X,Y),
    # the matrix indexing for `grid` is instead to be done as (Y,X).
    # Moreover, the Y is flipped in the map (see maps images), then for moving the
    # robot UP we actually need to increment Y (row_index + 1)
    # instead of decreasing it (such as it would happen in a normal matrix).

    if grid[gpose[1], gpose[0]] == obstacle_identifier:
        return False
    
    cells = set([gpose])
    # explore all adjacent cells for a maximum of `collision_expansion` layers
    for _ in range(collision_expansion):
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
            try:
                if grid[tup[1], tup[0]] == obstacle_identifier:
                    return False
            except IndexError:
                return False

        cells = cells.union(tmp)

    return True


def get_neighbourhood(grid, grid_pose):
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


def get_complete_path(nodes_set, node):
    path = [node.pose]
    while node.prev_node_pose != None:
        path.append(node.prev_node_pose)
        node = nodes_set[node.prev_node_pose]
    return path[::-1]


def path_planning(grid, grid_resolution, allow_unknown, 
                    start_odom, goal_odom, goal_identifier = None, 
                    obstacle_identifier = 100, unknown_identifier = -1, collision_expansion = 3):
    """ 
        Applies A* or Dijkstra depending on the fact that `goal_odom` is specified or not. 
        If not, then a `goal_identifier` (as integer) is needed for identifying what to look for in the grid. 
        Parameter `allow_unknown` set to True enables traversing unknown areas.
    """

    start_grid = odom_to_grid(start_odom, grid_resolution)
    goal_grid = None if goal_odom == None else odom_to_grid(goal_odom, grid_resolution)
    
    if goal_grid == None and goal_identifier == None: 
        raise Exception('You must have either a goal pose or goal_identifier.')

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
            return [grid_to_odom(p, grid_resolution) for p in get_complete_path(closed_set, current_node)] # success: goal reached

        del open_set[chosen_idx]
        closed_set[chosen_idx] = current_node

        if not allow_unknown and grid[current_node.pose[1], current_node.pose[0]] == unknown_identifier:
            continue # if traversing unknown areas is not allowed, then do not compute neighbourhood of unknown nodes

        neighbourhood = get_neighbourhood(grid, current_node.pose) # list of neighbours poses (tuple)
        
        for _, neighbour in enumerate(neighbourhood):
        
            if neighbour in closed_set:
                continue # already explored

            # cells near to the starting pose are not excluded from being a neighbour
            if not np.allclose(neighbour, start_node.pose, atol=2) and not is_valid_neighbour(grid, neighbour, obstacle_identifier, collision_expansion):
                continue # obstacle or out of the map

            g_score = current_node.g_score + move_utils.euclidean_distance_tuple(current_node.pose, neighbour)

            if neighbour not in open_set: # new node
                open_set[neighbour] = GraphNode(neighbour, g_score, current_node.pose, goal_grid)
            
            elif open_set[neighbour].g_score > g_score: # best path for reaching the node `neighbour` 
                    open_set[neighbour].g_score = g_score
                    open_set[neighbour].prev_node_pose = current_node.pose

    return [] # failure: open_set is empty but goal was never reached

