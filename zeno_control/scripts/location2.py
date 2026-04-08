#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.animation import FuncAnimation

from shapely.geometry import Polygon, MultiPoint, LineString, MultiLineString, Point
from shapely.affinity import rotate
from shapely.geometry.polygon import orient
from shapely.validation import explain_validity

from geometry_msgs.msg import Pose, Point as GeoPoint
from marta_msgs.msg import NavStatus
from artur_msgs.msg import Waypoint, WaypointList
from shapely.geometry import Polygon
from geodetic_functions import ll2ne, ne2ll


class MapVisualizer:
    def __init__(self):
        # Load configuration from YAML file
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)


        # Set the robot's initial position and origin
        self.robot_position = None 
        self.first_position_received = False
        
        self.origin_ll = (
            config['initial_position']['latitude'],
            config['initial_position']['longitude']
        )

        # Initialize path storage
        self.path = []

        # Convert exploration area coordinates to NED (North-East-Down) system
        self.polygon_coords_geo = [tuple(coord) for coord in config['exploration_area']]
        self.polygon_ned_coords = [ll2ne(self.origin_ll, (lat, lon)) for lon, lat in self.polygon_coords_geo]
        self.polygon_ned = orient(Polygon(self.polygon_ned_coords).convex_hull, sign=1.0)

        if not self.polygon_ned.is_valid:
            rospy.logerr("Invalid polygon: %s" % explain_validity(self.polygon_ned))
            return

        # Prepare plots
        self.setup_plot()

        # Generate the lawn mower path based on the configuration
        spacing = config['lawnmower']['spacing']
        direction = config['lawnmower']['direction']
        angle_deg = config['lawnmower']['angle_deg']
        reverse_lines = config['lawnmower'].get('reverse', False)
        self.lawn_mower_path_ned = self.generate_lawn_mower_path(
            self.polygon_ned, spacing=spacing, direction=direction,
            angle_deg=angle_deg, reverse_lines=reverse_lines
        )           

        # Visualize the path and waypoints
        self.visualize_path()

        # Save and publish waypoints
        self.save_waypoints_to_yaml("waypoint_zigzag.yaml")
        self.waypoint_list_pub = rospy.Publisher("waypoints", WaypointList, queue_size=10)
        self.waypoint_timer = rospy.Timer(rospy.Duration(2.0), self.publish_waypoints)
        self.publish_waypoints()

        # Subscribe to robot position updates
        rospy.Subscriber("/nav_status", NavStatus, self.update_position)

        # Start animation and show plots
        self.ani = FuncAnimation(self.fig, self.update_map, interval=1000)
        plt.show()
        self.publish_waypoints()

    def setup_plot(self):
        """Setup the initial plots for the lake area and exploration area."""
        # Get the path to the image relative to the package
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        img_path = os.path.join(pkg_path, 'scripts', 'Laghetti_di_Campo.png')
        self.fig, (self.ax_full, self.ax_zoom) = plt.subplots(1, 2, figsize=(14, 7))
        #img = mpimg.imread('/home/eugdeca/catkin_ws/src/zeno_control/scripts/Laghetti_di_Campo.png')
        img = mpimg.imread(img_path)
        
        # Bounding box della mappa in lat/lon
        min_lat, max_lat = 43.7011520, 43.7073568
        min_lon, max_lon = 10.4704373, 10.4768747
        self.ax_full.imshow(img, extent=[min_lon, max_lon, min_lat, max_lat], aspect='auto')
        
        self.ax_zoom.set_facecolor('lightblue')

        self.ax_full.set_title("Lake Area (Lon/Lat)")
        self.ax_full.set_xlabel("Longitude")
        self.ax_full.set_ylabel("Latitude")

        self.ax_zoom.set_title("Exploration Area (NED)")
        self.ax_zoom.set_xlabel("North (m)")
        self.ax_zoom.set_ylabel("East (m)")

        
        # Plot the exploration area in both geo-coordinates (Lon/Lat) and NED
        geo_polygon = [ne2ll(self.origin_ll, (n, e)) for n, e in self.polygon_ned.exterior.coords]
        px, py = zip(*[(lon, lat) for lat, lon in geo_polygon])
        self.ax_full.plot(px, py, color='orange', linewidth=2, label="Planned Area")

        npx, npy = zip(*self.polygon_ned.exterior.coords)
        self.ax_zoom.plot(npx, npy, color='orange', linewidth=2)

        # Initialize robot and path markers for both views
        self.robot_marker_full, = self.ax_full.plot([], [], marker='o', color='red', markersize=8)
        self.path_line_full, = self.ax_full.plot([], [], color='blue', linewidth=2)
        self.robot_marker_zoom, = self.ax_zoom.plot([], [], marker='o', color='red', markersize=8)
        self.path_line_zoom, = self.ax_zoom.plot([], [], color='blue', linewidth=2)

    def visualize_path(self):
        """Visualize the generated lawnmower path and waypoints."""
        if self.lawn_mower_path_ned:
            nx, ny = zip(*self.lawn_mower_path_ned)
            self.ax_zoom.plot(nx, ny, color='green', linestyle='--')

            # Convert NED path to geo-coordinates and plot
            geo_path = [ne2ll(self.origin_ll, ned) for ned in self.lawn_mower_path_ned]
            x_vals, y_vals = zip(*[(lon, lat) for lat, lon in geo_path])
            self.ax_full.plot(x_vals, y_vals, color='green', linestyle='--', label='Lawn Mower Path')

            # Prepare waypoints for publishing
            self.waypoints = [GeoPoint(x=n, y=e) for n, e in self.lawn_mower_path_ned]

            # Label the waypoints on the zoomed map
            for idx, ned in enumerate(self.lawn_mower_path_ned):
                self.ax_zoom.text(ned[0], ned[1], str(idx + 1), fontsize=8, color='black', ha='center', va='center')

            # Adjust zoom window to fit the path
            self.ax_zoom.set_xlim(min(nx) - 15, max(nx) + 15)
            self.ax_zoom.set_ylim(min(ny) - 15, max(ny) + 15)
            self.ax_full.legend()

    def publish_waypoints(self, event=None):
        """Publish waypoints to the '/waypoints' ROS topic."""
        waypoint_list_msg = WaypointList()
        for waypoint in self.waypoints:
            msg = Waypoint()
            msg.x = waypoint.x
            msg.y = waypoint.y
            waypoint_list_msg.waypoints.append(msg)

        waypoint_list_msg.received = True
        rospy.loginfo_throttle(5, "Publishing %d waypoints to /waypoints" % len(waypoint_list_msg.waypoints))
        self.waypoint_list_pub.publish(waypoint_list_msg)

    def update_position(self, msg):
        """Aggiorna la posizione del robot basata sul messaggio NavStatus."""
        
        # Estrai la latitudine e longitudine dal messaggio
        new_position = [msg.position.latitude, msg.position.longitude]
        
        # Se è la prima volta che riceviamo un messaggio, inizializziamo la posizione
        if self.robot_position is None:
            self.robot_position = new_position
            
        
        # Se la posizione è cambiata, aggiorniamo
        elif self.robot_position != new_position:
            self.previous_position = self.robot_position
            self.robot_position = new_position
            
        

        # Aggiungi la posizione alla lista del percorso
        self.path.append((self.robot_position[0], self.robot_position[1]))

    def update_map(self, frame):
        """Aggiorna la visualizzazione sulla mappa."""
        if self.robot_position is not None:
            # Aggiorna marker posizione attuale
            self.robot_marker_full.set_data(self.robot_position[1], self.robot_position[0])  # (lon, lat)
            ned_pos = ll2ne(self.origin_ll, (self.robot_position[0], self.robot_position[1]))
            self.robot_marker_zoom.set_data(ned_pos[0], ned_pos[1])

            # Aggiorna la traiettoria completa in geo (per la vista "full")
            x_vals, y_vals = zip(*[(lon, lat) for lat, lon in self.path])
            self.path_line_full.set_data(x_vals, y_vals)

            # Aggiorna la traiettoria in NED (per la vista zoom)
            ned_path = [ll2ne(self.origin_ll, (lat, lon)) for lat, lon in self.path]
            nx, ny = zip(*ned_path)
            self.path_line_zoom.set_data(nx, ny)

        return self.robot_marker_full, self.path_line_full, self.robot_marker_zoom, self.path_line_zoom

    def save_waypoints_to_yaml(self, filename="waypoint_zigzag.yaml"):
        """Save the waypoints to a YAML file."""
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        file_path = os.path.join(pkg_path, filename)

        rospy.loginfo("Saving waypoints to {}: {}".format(file_path, self.waypoints))
        data = {"waypoints": [{"x": float(wp.x), "y": float(wp.y)} for wp in self.waypoints]}

        try:
            with open(file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            rospy.loginfo("Waypoints saved to YAML: {}".format(filename))


        except Exception as e:
            rospy.logerr("Error saving waypoints to YAML: %s" % e)

    def generate_lawn_mower_path(self, polygon, spacing=0.0001, direction='horizontal', angle_deg=0, reverse_lines=False):
        """
        Generate a lawn mower path inside the given polygon.

        Args:
            polygon (shapely.geometry.Polygon): The area of interest.
            spacing (float): Distance between lines.
            direction (str): 'horizontal' or 'vertical' lawn mower direction.
            angle_deg (float): Rotation angle of the lawn mower path in degrees.
            reverse_lines (bool): If True, reverse the order of lines.

        Returns:
            List[Tuple[float, float]]: List of waypoints defining the path.
        """
        centroid = polygon.centroid
        rotated_polygon = rotate(polygon, angle_deg, origin=centroid, use_radians=False)

        minx, miny, maxx, maxy = rotated_polygon.bounds
        lines = []

        # Generate lines based on the specified direction
        if direction == 'horizontal':
            y = miny
            while y < maxy:
                line = LineString([(minx, y), (maxx, y)])
                clipped = rotated_polygon.intersection(line)
                if not clipped.is_empty:
                    lines.append(clipped)
                y += spacing
        elif direction == 'vertical':
            x = minx
            while x < maxx:
                line = LineString([(x, miny), (x, maxy)])
                clipped = rotated_polygon.intersection(line)
                if not clipped.is_empty:
                    lines.append(clipped)
                x += spacing
        else:
            rospy.logwarn("Unknown direction '%s' for lawn mower pattern." % direction)
            return []

        # Reverse lines if needed
        if reverse_lines:
            lines.reverse()
        path = []
        reverse = False

        # Process each line segment
        for segment in lines:
            if segment.is_empty:
                continue

            if isinstance(segment, MultiLineString):
                # Handle MultiLineString segments
                for sub_segment in segment.geoms:
                    coords = list(sub_segment.coords)
                    if reverse:
                        coords.reverse()
                    path.extend(coords)
                reverse = not reverse
            else:
                # Handle single LineString segments
                coords = list(segment.coords)
                if reverse:
                    coords.reverse()
                path.extend(coords)
                reverse = not reverse

        # If rotation was applied, undo it by rotating the path back
        if angle_deg != 0:
            rotated_path = [rotate(Point(x, y), -angle_deg, origin=centroid, use_radians=False) for x, y in path]
            path = [(p.x, p.y) for p in rotated_path]

        return path


if __name__ == '__main__':
    rospy.init_node('map_visualizer')
    MapVisualizer()
    rospy.spin()