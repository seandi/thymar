#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
from math import pi
import numpy as np
from matplotlib import pyplot as plt
from thymar_controller import ThymarController


import movement_utils as mv
from explorer_controller import ExplorerController

class Target:
    def __init__(self,x,y,radius):
        self.x = x
        self.y = y
        self.radius = radius



class Thymar:
    def __init__(self, rate=10 ):
        rospy.init_node('Thymar', anonymous=True)
        self.rate = rospy.Rate(rate)

        self.name = rospy.get_param("~name")

        print("Process for "+self.name+" has started!")

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

        self.position = Point()
        self.orientation = 0
        self.proximity = [0.12] * 7
        self.covariance = None
        self.target_found = False
        self.target = {}
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
        self.target = Target(data.pose.position.x,data.pose.position.y,data.pose.position.z)
        self.target_found = True
        print("Target found in ({0},{1}) with radius {2}!".format(self.target.x,self.target.y,self.target.radius))

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
            rospy.loginfo('target = x:' + str(self.target.x) + ', y:' + str(self.target.y))
        np.save('map.npy', self.occupancy_grid)
        
    def on_exit(self):
        self.stop()
        self.log()


    def run(self):
        # self.controller = ThymarController(self.grid_resolution)
        self.controller = ExplorerController()
        plt.figure(figsize = (2,2))
        
        while not rospy.is_shutdown():
            vel = self.controller.run(self.proximity, self.position, self.orientation)

            # Plots the map
            if(self.occupancy_grid.shape[0] > 1 and self.plot_grid):
                plt.imshow(np.flipud(self.occupancy_grid), interpolation='nearest')
                plt.pause(0.05)

            self.velocity_publisher.publish(vel)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        thymar= Thymar()
        thymar.run()
    except rospy.ROSInterruptException as e:
        pass
