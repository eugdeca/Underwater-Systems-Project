#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from PIL import Image, ImageDraw
import yaml
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from std_msgs.msg import Bool

def generate_map(width=500, height=500, resolution=0.05, obstacles=None, polygons=None, map_name='enu_map'):
    """
    Generates a 2D map in ENU coordinates (East-North-Up).
    Saves .pgm and .yaml and publishes as OccupancyGrid.
    """
    # Create white image (255 = free space)
    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    if obstacles is None:
        obstacles = [
            {"x": 100, "y": 100, "radius": 30},
            {"x": 250, "y": 250, "radius": 40},
            {"x": 400, "y": 100, "radius": 10},
            {"x": 160, "y": 180, "radius": 10}
        ]

    for obs in obstacles:
        x = obs["x"]
        y = obs["y"]
        r = obs["radius"]
        draw.ellipse([(x - r, y - r), (x + r, y + r)], fill=0)

    # Draw polygon obstacles
    if polygons is None:
        polygons = [
            {"points": [(300, 300), (350, 320), (330, 370), (280, 360)]},
            {"points": [(100, 400), (150, 420), (130, 470), (80, 460)]}
        ]

    for poly in polygons:
        draw.polygon(poly["points"], fill=0)

    # ✅ Flip vertically to convert image coordinates (top-left origin) to ENU (bottom-left origin)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)

    # Save .pgm
    pgm_filename = "{}.pgm".format(map_name)
    image.save(pgm_filename)

    # Save .yaml metadata
    yaml_data = {
        "image": pgm_filename,
        "resolution": resolution,
        "origin": [-1.0, 1.0, 0.0],  # ENU: origin at bottom-left
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }

    yaml_filename = "{}.yaml".format(map_name)
    with open(yaml_filename, 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file)

    rospy.loginfo("Map saved as {} and {}".format(pgm_filename, yaml_filename))
    return image, resolution, width, height

def image_to_occupancy_grid(image, resolution, width, height):
    data = list(image.getdata())
    grid = []

    for val in data:
        if val == 0:
            grid.append(100)
        elif val == 255:
            grid.append(0)
        else:
            grid.append(-1)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "map"  # ✅ Use ENU frame

    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.origin = Pose(
        position=Point(x=-1.0, y=1.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    occupancy_grid.data = grid
    return occupancy_grid

def main():
    rospy.init_node('enu_map_generator_node')
    pub = rospy.Publisher('projected_map', OccupancyGrid, queue_size=1, latch=True)
    rospy.loginfo("Generating ENU-style 2D map...")
    pub_ready = rospy.Publisher('map_ready', Bool, queue_size=1, latch=True)

    

    image, resolution, width, height = generate_map()
    msg = image_to_occupancy_grid(image, resolution, width, height)

    pub.publish(msg)
    
    rospy.loginfo("Published map on /project_map using ENU coordinates")

    #Publish map_ready signal
    rospy.sleep(0.5)  # small delay to ensure /project_map is received first
    pub_ready.publish(Bool(data=True))
    rospy.loginfo("Published map_ready = True on /map_ready")

    rospy.spin()

if __name__ == '__main__':
    main()
