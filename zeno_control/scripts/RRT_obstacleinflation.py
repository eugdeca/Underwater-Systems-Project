#!/usr/bin/env python
import rospy
import os
import rospkg
import yaml
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import random
import math
from scipy.ndimage import binary_dilation
from skimage.morphology import disk
from marta_msgs.msg import NavStatus
from marta_msgs.msg import Position 
from artur_msgs.msg import Waypoint, WaypointList
from geodetic_functions import ll2ne
from scipy.interpolate import splprep, splev
from scipy.ndimage import median_filter
from std_msgs.msg import Bool


class RRTPlanner:
    def __init__(self):
        # Load config
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        rospy.init_node('rrt_planner_node')
        

        # Flag to start after map is ready
        self.map_ready = False

        # Wait for map_ready to be True
        rospy.Subscriber('map_ready', Bool, self.flag)

    def flag(self, msg):
        if msg.data:  # True received
            if not self.map_ready:
                #rospy.Subscriber('/project_map', OccupancyGrid, self.map_callback)
                self.map_sub = rospy.Subscriber('projected_map', OccupancyGrid, self.map_callback)

                self.waypoints_pub = rospy.Publisher('waypoints2', WaypointList, queue_size=10)
                self.nav_status_sub = rospy.Subscriber('/nav_status', NavStatus, self.nav_status_callback)
                self.sphere_pos_sub = rospy.Subscriber('sphere_position', Position, self.sphere_position_callback)

            

                self.goal_position = None
                self.gps_start = None
                self.ll0 = (
                    self.config['initial_position']['latitude'],
                    self.config['initial_position']['longitude']
                )
                self.final_path = []
                self.map_received = False

                rospy.loginfo("RRT Planner node initialized.")


    def nav_status_callback(self, msg):
        if self.gps_start is not None:
            return

        lat, lon = msg.position.latitude, msg.position.longitude
        n, e = ll2ne(self.ll0, (lat, lon))
        self.gps_start = (e, n)
        rospy.loginfo("Received GPS position: lat={}, lon={} -> start=({:.2f}, {:.2f})".format(lat, lon, e, n))

        self.nav_status_sub.unregister()

    def sphere_position_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        n, e = ll2ne(self.ll0, (lat, lon))
        self.goal_position = (e, n)
        rospy.loginfo("Received sphere position: lat={}, lon={} -> goal=({:.2f}, {:.2f})".format(lat, lon, e, n))
        self.sphere_pos_sub.unregister()

        if self.map_received and self.gps_start is not None:
            self.run_rrt()

    def map_callback(self, msg):
        rospy.loginfo("Map received.")
        self.map = msg
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin.position

        data = np.array(msg.data).reshape((self.height, self.width))
        #self.map_array = np.where(data == 100, 1, 0).astype(np.uint8)
        self.map_array = np.where(data != 0, 1, 0).astype(np.uint8)
        self.map_array = median_filter(self.map_array, size = 3)

        self.inflate_obstacles()
        self.map_received = True
        self.map_sub.unregister()

        if self.gps_start is not None and self.goal_position is not None:
            self.run_rrt()

    def inflate_obstacles(self):
        rospy.loginfo("Inflating obstacles by {} meter.".format(self.config['inflation']))
        robot_radius = self.config['inflation']
        pixels = int(robot_radius / self.resolution)
        structuring_element = disk(pixels)

        temp_map = self.map_array.copy()
       

        # Inflate obstacles from the updated map
        self.inflated_map = binary_dilation(temp_map, structure=structuring_element).astype(np.uint8)



    def is_free(self, pt):
        x, y = pt
        xi = int((x - self.origin.x) / self.resolution)
        yi = int((y - self.origin.y) / self.resolution)
        if 0 <= xi < self.width and 0 <= yi < self.height:
            return self.inflated_map[yi, xi] == 0
        return False


    def nearest_free_point(self, x, y, max_radius=100.0):
        step = self.resolution
        for r in np.arange(step, max_radius, step):
            for angle in np.linspace(0, 2 * np.pi, int(2 * np.pi * r / step)):
                nx = x + r * math.cos(angle)
                ny = y + r * math.sin(angle)
                if self.is_free((nx, ny)):
                    return (nx, ny)
        return None

    def run_rrt(self):
        rospy.loginfo("Running RRT...")

        if self.gps_start is None:
            rospy.logwarn("Waiting for GPS start position from /nav_status...")
            return

        start = self.gps_start
        goal = self.goal_position

        if not self.is_free(start):
            rospy.logwarn("Start is in obstacle, finding nearest free point...")
            start = self.nearest_free_point(*start)
        if not self.is_free(goal):
            rospy.logwarn("Goal is in obstacle, finding nearest free point...")
            goal = self.nearest_free_point(*goal)

        max_iter = 20000
        step_size = 0.5
        goal_radius = 0.75

        nodes = [start]
        parents = {start: None}

        for _ in range(max_iter):
            rand = (
                random.uniform(0, self.width * self.resolution),
                random.uniform(0, self.height * self.resolution)
            )

            nearest = min(nodes, key=lambda n: math.hypot(n[0] - rand[0], n[1] - rand[1]))
            theta = math.atan2(rand[1] - nearest[1], rand[0] - nearest[0])
            new_point = (
                nearest[0] + step_size * math.cos(theta),
                nearest[1] + step_size * math.sin(theta)
            )

            if not self.is_free(new_point):
                continue

            if self.check_collision(nearest, new_point):
                nodes.append(new_point)
                parents[new_point] = nearest

                if math.hypot(new_point[0] - goal[0], new_point[1] - goal[1]) < goal_radius:
                    rospy.loginfo("Goal reached!")
                    parents[goal] = new_point
                    self.publish_path(parents, goal)
                    self.visualize(nodes, parents, start, goal)
                    return

        rospy.logwarn("RRT failed to find a path.")
        self.visualize(nodes, parents, start, goal, success=False)

    def check_collision(self, p1, p2, samples=10):
        for i in range(samples + 1):
            x = p1[0] + (p2[0] - p1[0]) * i / samples
            y = p1[1] + (p2[1] - p1[1]) * i / samples
            if not self.is_free((x, y)):
                return False
        return True

    def smooth_path_with_bspline(self, path, smooth_factor=3, max_attempts=5):
        if len(path) < 4:
            return path

        x, y = zip(*path)
        for _ in range(max_attempts):
            try:
                tck, _ = splprep([x, y], s=smooth_factor)
                u_fine = np.linspace(0, 1, len(path) * 10)
                x_smooth, y_smooth = splev(u_fine, tck)
                smooth_path = list(zip(x_smooth, y_smooth))

                if all(self.is_free(pt) for pt in smooth_path):
                    rospy.loginfo("B-spline smoothing succeeded with s={}".format(smooth_factor))
                    return smooth_path
                else:
                    rospy.logwarn("B-spline intersects obstacles at s={}, reducing smoothing...".format(smooth_factor))
                    smooth_factor *= 0.6
            except Exception as e:
                rospy.logerr("Spline fitting failed: {}".format(e))
                break

        rospy.logwarn("B-spline smoothing failed, returning original path.")
        return path

    def publish_path(self, parents, goal):
        rospy.loginfo("Publishing waypoint list as a single message...")
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parents.get(node)

        path = list(reversed(path))
        smoothed_path = self.smooth_path_with_bspline(path, smooth_factor=3.0)
        self.final_path = smoothed_path

        msg = WaypointList()
        msg.waypoints = []
        for waypoint in smoothed_path:
            wp = Waypoint()
            wp.x = waypoint[0]
            wp.y = waypoint[1]
            msg.waypoints.append(wp)
        
        self.waypoints_pub.publish(msg)
        rospy.loginfo("Published {} waypoints in a single message.".format(len(msg.waypoints)))
    
    def visualize(self, nodes, parents, start, goal, success=True):
        plt.figure(figsize=(8, 8))

        res = self.resolution
        origin_x = self.origin.x
        origin_y = self.origin.y

        im = plt.imshow(self.inflated_map, cmap='gray', origin='lower')

        # Set axis ticks and labels to reflect real-world coordinates
        ax = plt.gca()
        ax.set_xlim(0, self.inflated_map.shape[1])
        ax.set_xticks(np.linspace(0, self.inflated_map.shape[1], num=6))
        ax.set_xticklabels(["{:.1f}".format(origin_x + i * res * (self.inflated_map.shape[1] / 5)) for i in range(6)])

        ax.set_ylim(0, self.inflated_map.shape[0])
        ax.set_yticks(np.linspace(0, self.inflated_map.shape[0], num=6))
        ax.set_yticklabels(["{:.1f}".format(origin_y + i * res * (self.inflated_map.shape[0] / 5)) for i in range(6)])

        # Convert real-world coordinates to pixel coordinates for plotting
        def world_to_pixel(x, y):
            px = (x - origin_x) / res
            py = (y - origin_y) / res
            return px, py

        for node, parent in parents.items():
            if parent is not None:
                n1 = world_to_pixel(*node)
                n2 = world_to_pixel(*parent)
                plt.plot([n1[0], n2[0]], [n1[1], n2[1]], color='blue', linewidth=0.5)

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = parents.get(node)
        path = list(reversed(path))

        if success and path:
            x_raw, y_raw = zip(*[world_to_pixel(*pt) for pt in path])
            plt.plot(x_raw, y_raw, '--', color='gray', label='Original Path')

        if success and self.final_path:
            x_smooth, y_smooth = zip(*[world_to_pixel(*pt) for pt in self.final_path])
            plt.plot(x_smooth, y_smooth, color='green', linewidth=2, label='Smoothed Path')

        start_px = world_to_pixel(*start)
        goal_px = world_to_pixel(*goal)
        plt.scatter(*start_px, color='red', s=50, label='Start')
        plt.scatter(*goal_px, color='orange', s=50, label='Goal')

        plt.title('RRT Path Planning with B-Spline Smoothing')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        
 


if __name__ == '__main__':
    planner = RRTPlanner()
    rospy.spin()
