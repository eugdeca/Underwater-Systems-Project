#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import rospkg
import yaml
import threading
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from marta_msgs.msg import NavStatus  # Replace with your actual message name
from geometry_msgs.msg import Point
from geodetic_functions import ll2ne
from matplotlib.patches import Circle
from artur_msgs.msg import WaypointList  # Replace with your actual WaypointList type
from std_msgs.msg import Bool


class MapPositionPlotter:
    """
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        rospy.init_node('map_position_plotter', anonymous=True)
        self.ll0 = (
            self.config['initial_position']['latitude'],
            self.config['initial_position']['longitude']
        )

        self.map_data = None
        self.map_info = None
        self.position = None

        self.map_lock = threading.Lock()
        self.pos_lock = threading.Lock()
        self.bool = rospy.Subscriber('/map_ready', Bool, self.flag)

        if self.bool:
            rospy.Subscriber('/project_map', OccupancyGrid, self.map_callback)
            rospy.Subscriber('/nav_status', NavStatus, self.position_callback)
            rospy.Subscriber('waypoints2', WaypointList, self.waypoints_callback)

            self.thread = threading.Thread(target=self.plot_loop)
            self.thread.start()
            self.path_pixels = []
            self.waypoints = []
        else:
            return

    def flag(self, msg):
        self.bool = msg
        print(self.bool)
        """

    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        rospy.init_node('map_position_plotter', anonymous=True)
        self.ll0 = (
            self.config['initial_position']['latitude'],
            self.config['initial_position']['longitude']
        )

        self.map_data = None
        self.map_info = None
        self.position = None

        self.map_lock = threading.Lock()
        self.pos_lock = threading.Lock()

        self.path_pixels = []
        self.waypoints = []

        # Flag to start after map is ready
        self.map_ready = False

        # Wait for map_ready to be True
        rospy.Subscriber('map_ready', Bool, self.flag)

    def flag(self, msg):
        if msg.data:  # True received
            if not self.map_ready:
                self.map_ready = True
                print("Map is ready! Starting other components...")

                rospy.Subscriber('projected_map', OccupancyGrid, self.map_callback)
                rospy.Subscriber('/nav_status', NavStatus, self.position_callback)
                rospy.Subscriber('waypoints2', WaypointList, self.waypoints_callback)

                self.thread = threading.Thread(target=self.plot_loop)
                self.thread.start()
                #self.map_ready = True

    def map_callback(self, msg):
        with self.map_lock:
            self.map_info = msg.info
            data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
            self.map_data = data

    def position_callback(self, msg):
        with self.pos_lock:
            self.position = msg.position

            if self.map_info:
                res = self.map_info.resolution
                origin_x = self.map_info.origin.position.x
                origin_y = self.map_info.origin.position.y
                pos_lat = self.position.latitude
                pos_long = self.position.longitude
                pos = ll2ne(self.ll0, (pos_lat, pos_long))
                pos_x = pos[1]
                pos_y = pos[0]

                px = (pos_x - origin_x) / res
                py = (pos_y - origin_y) / res

                self.path_pixels.append((px, py))

    def waypoints_callback(self, msg):
        with self.map_lock:
            self.waypoints = []
            res = self.map_info.resolution
            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y

            for wp in msg.waypoints:
                px = (wp.x - origin_x) / res
                py = (wp.y - origin_y) / res
                self.waypoints.append((px, py))

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 8))
        im = None
        point_plot = None
        circle_patch = None
        waypoint_plot = None
        line_plot = None

        while not rospy.is_shutdown():
            if self.map_data is None or self.position is None:
                rospy.sleep(0.1)
                continue

            with self.map_lock:
                map_data = np.copy(self.map_data)
                res = self.map_info.resolution
                origin_x = self.map_info.origin.position.x
                origin_y = self.map_info.origin.position.y

            with self.pos_lock:
                pos_lat = self.position.latitude
                pos_long = self.position.longitude
                pos = ll2ne(self.ll0, (pos_lat, pos_long))
                pos_x = pos[1]
                pos_y = pos[0]

            if im is None:
                im = ax.imshow(map_data, cmap='gray', origin='lower')
                point_plot, = ax.plot([], [], 'ro', markersize=6)
                line_plot, = ax.plot([], [], 'r-', linewidth=1)
                circle_patch = Circle((0, 0), radius=0.5 / res, color='blue', fill=False, linewidth=1.5)
                ax.add_patch(circle_patch)
                waypoint_plot, = ax.plot([], [], 'g-', linewidth=2, label='RRT B-spline')

                ax.set_title("Real-time Map & Position")
                ax.set_xlabel("East")
                ax.set_ylabel("North")

            im.set_data(map_data)

            px = (pos_x - origin_x) / res
            py = (pos_y - origin_y) / res

            point_plot.set_data(px, py)
            circle_patch.center = (px, py)

            ax.set_xlim(0, map_data.shape[1])
            ax.set_xticks(np.linspace(0, map_data.shape[1], num=6))
            ax.set_xticklabels(["{:.1f}".format(origin_x + i * res * (map_data.shape[1] / 5)) for i in range(6)])
            ax.set_ylim(0, map_data.shape[0])
            ax.set_yticks(np.linspace(0, map_data.shape[0], num=6))
            ax.set_yticklabels(["{:.1f}".format(origin_y + i * res * (map_data.shape[0] / 5)) for i in range(6)])

            if self.path_pixels:
                path_xs, path_ys = zip(*self.path_pixels)
                line_plot.set_data(path_xs, path_ys)

            if self.waypoints:
                wx, wy = zip(*self.waypoints)
                waypoint_plot.set_data(wx, wy)

            plt.pause(0.05)

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    try:
        MapPositionPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
