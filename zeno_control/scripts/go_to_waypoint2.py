#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import os
import rospkg
import yaml
import numpy as np
from math import atan2, degrees, sin, cos
from geopy import Point

from marta_msgs.msg import Position
from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from artur_msgs.msg import Waypoint, WaypointList  # Use the correct list message type
from geodetic_functions import ll2ne
from math import exp


class WaypointFollower:
    def __init__(self):
        # Load configuration
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        rospy.init_node("go_to_waypoints")
        self.pub = rospy.Publisher("/relative_error", Rel_error_joystick, queue_size=1)
        self.sub = rospy.Subscriber("/nav_status", NavStatus, self.nav_cb)
        
        self.waypoint_list_sub = rospy.Subscriber("waypoints2", WaypointList, self.waypoint_list_callback)
        self.sphere_pos_sub = rospy.Subscriber('sphere_position', Position, self.sphere_position_callback)
        rospy.on_shutdown(self.handle_shutdown)
        
        self.ll0 = (
            config['initial_position']['latitude'],
            config['initial_position']['longitude']
        )
        self.goal_position = None
        self.max_speed = config['motion_params']['max_speed']
        self.stop_threshold = config['motion_params']['stop_threshold']
        self.yaw_threshold = config['motion_params']['yaw_threshold']

        self.current_waypoint_index = 0
        self.reached_current = False

        self.initial_point = Point((config['initial_position']['latitude'], config['initial_position']['longitude']))
        self.waypoints = []
        self.waypoints_received = False

        rospy.loginfo("Waiting for waypoint list...")
        rospy.spin()

    def waypoint_list_callback(self, msg):
        if not self.waypoints_received:
            if msg.waypoints:  # make sure the list is not empty
                first_wp = msg.waypoints[0]
                rospy.loginfo("First waypoint: east={}, north={}".format(first_wp.x, first_wp.y))

        self.waypoints = [(wp.y, wp.x) for wp in msg.waypoints]
        self.waypoints_received = True
        self.current_waypoint_index = 0
        self.reached_current = False
        rospy.loginfo("Received {} waypoints. Starting navigation.".format(len(self.waypoints)))

    def nav_cb(self, msg):
        if not self.waypoints_received:
            return
        
        origin_ll = (self.initial_point.latitude, self.initial_point.longitude)
        current_ll = (msg.position.latitude, msg.position.longitude)
        current_ne = ll2ne(origin_ll, current_ll)

        current_yaw = msg.orientation.yaw

        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached.")
            self.stop_target(current_ne, current_yaw)

            return

        
        target_ne = self.waypoints[self.current_waypoint_index]
        goal_ne = self.waypoints[-1]

        delta_n = target_ne[0] - current_ne[0]
        delta_e = target_ne[1] - current_ne[1]
        distance_to_wp = np.hypot(delta_n, delta_e)

        delta_goal_n = goal_ne[0] - current_ne[0]
        delta_goal_e = goal_ne[1] - current_ne[1]
        distance_to_goal = np.hypot(delta_goal_n, delta_goal_e)

        desired_yaw = atan2(delta_e, delta_n)
        yaw_error = degrees(atan2(sin(desired_yaw - current_yaw), cos(desired_yaw - current_yaw)))

        cmd = Rel_error_joystick()

        if distance_to_wp < self.stop_threshold:
            if not self.reached_current:
                rospy.loginfo("Reached waypoint {}".format(self.current_waypoint_index + 1))
                self.stop_vehicle()
                self.reached_current = True
            else:
                self.current_waypoint_index += 1
                self.reached_current = False
        else:
            if distance_to_goal > 2:
                if abs(yaw_error) > self.yaw_threshold:
                    if abs(yaw_error) < 45 and distance_to_wp < 3*self.stop_threshold:
                        surge_speed = 0.2
                        yaw_control = yaw_error
                    else:
                        surge_speed = 0
                        yaw_control = yaw_error
                else:
                    surge_speed = min(distance_to_wp * 0.5, self.max_speed)
                    yaw_control = 0
            else:
                rospy.loginfo("I am slowing down")
                if abs(yaw_error) > self.yaw_threshold:
                    if abs(yaw_error) < 45 and distance_to_wp < 3*self.stop_threshold:
                        surge_speed = 0.2 * exp(distance_to_goal) / 7
                        yaw_control = yaw_error
                    else:
                        surge_speed = 0
                        yaw_control = yaw_error
                else:
                    surge_speed = min(distance_to_wp * 0.5, self.max_speed)
                    yaw_control = 0

            cmd.error_yaw = yaw_control
            cmd.error_surge_speed = surge_speed
            self.pub.publish(cmd)
            
            rospy.loginfo_throttle(1, "WP {} | Distance: {:.2f} m | Yaw error: {:.2f}° | Surge: {:.2f} m/s".format(
                self.current_waypoint_index + 1, distance_to_wp, yaw_error, surge_speed))

    def sphere_position_callback(self, msg):        
        lat, lon = msg.latitude, msg.longitude
        n, e = ll2ne(self.ll0, (lat, lon))
        self.goal_position = (e, n)
        rospy.loginfo("Received sphere position: lat={}, lon={} -> goal=({:.2f}, {:.2f})".format(lat, lon, e, n))
        self.sphere_pos_sub.unregister()


    def stop_vehicle(self):
        stop_msg = Rel_error_joystick()
        self.pub.publish(stop_msg)
        rospy.loginfo("Vehicle stopped.")


    def stop_target(self, current_ne, current_yaw):
        if self.goal_position is None:
            rospy.logwarn("Goal position not set yet.")
            return

        # self.goal_position è (e, n)
        delta_target_n = self.goal_position[1] - current_ne[0]  # n - n
        delta_target_e = self.goal_position[0] - current_ne[1]  # e - e

        target_yaw = atan2(delta_target_e, delta_target_n)
        yaw_error = degrees(atan2(sin(target_yaw - current_yaw), cos(target_yaw - current_yaw)))

        cmd = Rel_error_joystick()
        cmd.error_yaw = yaw_error
        cmd.error_surge_speed = 0
        self.pub.publish(cmd)
        rospy.loginfo("Aligning with goal: Yaw error {:.2f}°".format(yaw_error))
    """

    def stop_target(self, current_ne):
        delta_target_n = self.goal_position_ne[0] - current_ne[0]
        delta_target_e = self.goal_position_ne[1] - current_ne[1]
        # distance_to_target = np.hypot(delta_target_n, delta_target_e)

        target_yaw = atan2(delta_target_e, delta_target_n)
        current_yaw = msg.orientation.yaw
        yaw_error = degrees(atan2(sin(target_yaw - current_yaw), cos(target_yaw - current_yaw)))


        
        stop_msg = Rel_error_joystick()
        cmd.error_yaw = yaw_error
        cmd.error_surge_speed = 0
        self.pub.publish(stop_msg)
        rospy.loginfo("Vehicle stopped.")
    """
    def handle_shutdown(self):
        rospy.logwarn("Shutting down: sending stop command.")
        self.stop_vehicle()



if __name__ == "__main__":
    try:
        WaypointFollower()
    except rospy.ROSInterruptException:
        pass

