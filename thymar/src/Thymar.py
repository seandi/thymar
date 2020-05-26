#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
import time
from math import pi
import numpy as np
from matplotlib import pyplot as plt

from thymar_controller import ThymarController
import movement_utils as mv
from movement_utils import Pose2D, Target, Status
from explorer_controller import ExplorerController




class Thymar:
    
    def __init__(self, rate=10 ):
        
        rospy.init_node('Thymar', anonymous=True)
        
        self.rate = rospy.Rate(rate)
        self.name = rospy.get_param("~name")

        print("Node for " + self.name + " initialized")

        self.velocity_publisher = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.update_pose)

        rospy.on_shutdown(self.on_exit) # tell ros what to do when CTRL+C is pressed

        prx = '/' + self.name + '/proximity/'
        self.proximity_sensors_name = [
            prx + 'left',
            prx + 'center_left',
            prx + 'center',
            prx + 'center_right',
            prx + 'right',
            prx + 'rear_left',
            prx + 'rear_right'
        ]

        callback = [self.update_proximity_left,
                    self.update_proximity_center_left,
                    self.update_proximity_center,
                    self.update_proximity_center_right,
                    self.update_proximity_right,
                    self.update_proximity_rear_left,
                    self.update_proximity_rear_right
                    ]

        self.proximity_subscribers = [
            rospy.Subscriber(sensor, Range, callback[i])
            for i, sensor in enumerate(self.proximity_sensors_name)
        ]

        self.occupancy_grid_subscriber = rospy.Subscriber('/' + self.name + '/occupancy_grid', OccupancyGrid, self.update_occupancy_grid)
        self.target_marker_subscriber = rospy.Subscriber('/' + self.name + '/target_marker', Marker, self.update_target)

        self.plan_publisher = rospy.Publisher('/' + self.name + '/plan', Path, queue_size=10)

        self.position = Point()
        self.orientation = 0
        self.proximity = [0.12] * 7
        self.covariance = None
        self.target_found = False
        self.target = Target()
        self.grid_width = None
        self.grid_height = None
        self.grid_resolution = None
        self.occupancy_grid = np.array([-1.])
        self.plot_grid = True

    def update_pose(self, data):
        self.position = data.pose.pose.position
        self.covariance = data.pose.covariance
        quaternion = data.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.orientation = euler_from_quaternion(explicit_quat)
        self.orientation = mv.to_positive_angle(self.orientation)

    def update_occupancy_grid(self, data):
        self.grid_width = data.info.width
        self.grid_height = data.info.height
        self.grid_resolution = data.info.resolution

        self.occupancy_grid = np.array(data.data)
        self.occupancy_grid = self.occupancy_grid.reshape(self.grid_height,self.grid_width)

    def update_target(self,data):
        pose = Pose2D(data.pose.position.x,data.pose.position.y)
        radius = data.pose.position.z
        self.target = Target(pose, radius)
        if not self.target_found: # only works the first time the target is found
            rospy.loginfo("Target found in ({0},{1}) with radius {2}!".format(self.target.pose.x,self.target.pose.y,self.target.radius))
        self.target_found = True

    def update_proximity_left(self, data):
        self.proximity[0] = data.range

    def update_proximity_center_left(self, data):
        self.proximity[1] = data.range

    def update_proximity_center(self, data):
        self.proximity[2] = data.range

    def update_proximity_center_right(self, data):
        self.proximity[3] = data.range

    def update_proximity_right(self, data):
        self.proximity[4] = data.range

    def update_proximity_rear_left(self, data):
        self.proximity[5] = data.range

    def update_proximity_rear_right(self, data):
        self.proximity[6] = data.range

    def stop(self):
        self.velocity_publisher.publish(Twist())  # set velocities to 0
        self.rate.sleep()

    def log(self):
        rospy.loginfo('')
        rospy.loginfo('grid resol = ' + str(self.grid_resolution))
        rospy.loginfo('grid shape = ' + str(self.occupancy_grid.shape))
        rospy.loginfo('position = ' + str(self.position))
        rospy.loginfo('orientation = ' + str(self.orientation))
        if self.target_found:
            rospy.loginfo('target = x:' + str(self.target.pose.x) + ', y:' + str(self.target.pose.y))
        np.save('map.npy', self.occupancy_grid)
        
    def on_exit(self):
        self.stop()
        # self.log()

    def ready(self):
        return self.position and self.orientation and self.grid_resolution

    def publish_path_plan(self, plan):
        if plan != None and len(plan) > 2:
            path = Path()
            path.header.frame_id = '/' + self.name + '/odom'
            path.header.stamp = rospy.get_rostime()
            for p in plan:
                pose = PoseStamped()
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                path.poses.append(pose)
            self.plan_publisher.publish(path)
            self.rate.sleep()


    def run(self):

        print('Waiting for subscribers to be ready...')
        while(not thymar.ready()):
            pass

        time.sleep(1)

        controller = ThymarController(self.grid_resolution,
                        initital_status = Status.EXPLORING_SMART,
                        status_after_founding_target = Status.CHASING_TARGET,
                        status_after_reaching_target = Status.EXPLORING_COVERAGE,
                        status_after_mapcoverage = Status.RETURNING,
                        allow_unknown_traversing = False)

        print('Controller is running! \n')

        while not rospy.is_shutdown() and not controller.status == Status.END:
            if self.target_found:
                controller.target_found = True
                controller.target = self.target

            vel = controller.run(self.position, self.orientation, self.proximity, self.occupancy_grid)
            self.velocity_publisher.publish(vel)
            self.rate.sleep()

            self.publish_path_plan(controller.planning_path)

        print('Controller stopped!')

        
        # self.controller = ExplorerController()
        # plt.figure(figsize = (2,2))

        # while not rospy.is_shutdown():
        #     vel = self.controller.run(self.proximity, self.position, self.orientation)

        #     # Plots the map
        #     if(self.occupancy_grid.shape[0] > 1 and self.plot_grid):
        #         plt.imshow(np.flipud(self.occupancy_grid), interpolation='nearest',origin='lower')
        #         plt.pause(0.05)

        #     self.velocity_publisher.publish(vel)
        #     self.rate.sleep()


if __name__ == '__main__':
    try:
        thymar = Thymar()
        thymar.run()

    except rospy.ROSInterruptException as e:
        pass
