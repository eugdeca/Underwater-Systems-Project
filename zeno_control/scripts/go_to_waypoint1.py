#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import yaml
import os
import rospkg
from math import atan2
from geopy import Point
from geodetic_functions import ll2ne
import time

from marta_msgs.msg import NavStatus
from artur_msgs.msg import Waypoint, WaypointList
from joystick_command.msg import Rel_error_joystick
from std_msgs.msg import Bool, Float32 
from math import exp


class WaypointFollower(object):
    def __init__(self):
        # Load configuration
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control')
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        rospy.init_node("go_to_waypoints")
        """
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('zeno_control') 
        config_path = os.path.join(pkg_path, 'config', 'config_mappa.yaml')
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            """
        rospy.on_shutdown(self.handle_shutdown)

        self.got_waypoints = False
        self.mission_started = False
        self.mission_stopped = False
        self.mission_start_time = None
        self.mission_duration_timer = None
        self.mission_duration_final = None
        self.waypoints = []
        self.robot_path = []
        self.reached_waypoints = []
        self.elapsed_mission_time = 0.0
        self.max_speed = config['motion_params']['max_speed']
        self.stop_threshold = config['motion_params']['stop_threshold']
        self.yaw_threshold = config['motion_params']['yaw_threshold']
        self.lawnmower_config = config.get('lawnmower', {})

        self.current_waypoint_index = 0
        self.reached_current = False

        self.sub_wp = rospy.Subscriber("waypoints", WaypointList, self.waypoint_cb)
        self.sub = rospy.Subscriber("/nav_status", NavStatus, self.nav_cb)
        self.sub_stop = rospy.Subscriber("stop_mission", Bool, self.stop_cb)

        self.pub = rospy.Publisher("/relative_error", Rel_error_joystick, queue_size=1)
        self.mission_done_pub = rospy.Publisher("exploration_mission_completed", Bool, queue_size=1)
        self.mission_start_pub = rospy.Publisher("exploration_mission_started", Bool, queue_size=1)
        self.mission_time_pub = rospy.Publisher("mission_time", Float32, queue_size=1) 
        
        self.initial_point = Point(
            config['initial_position']['latitude'],
            config['initial_position']['longitude']
        )
       
        

        rospy.loginfo("Waiting for waypoints on /waypoints...")

    def nav_cb(self, msg):
        if self.mission_stopped:
            return

        if not self.got_waypoints:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached.")
            if self.mission_duration_final is None and self.mission_start_time is not None:
                self.mission_duration_final = time.time() - self.mission_start_time

            self.stop_vehicle()
            self.mission_done_pub.publish(Bool(data=True))
            self.pub.unregister()
            
          

            if self.mission_duration_timer:
                self.mission_duration_timer.shutdown()
                self.mission_duration_timer = None
            return

        origin_ll = (self.initial_point.latitude, self.initial_point.longitude)
        current_ll = (msg.position.latitude, msg.position.longitude)
        current_ne = ll2ne(origin_ll, current_ll)
        goal_ne = self.waypoints[-1]

        target_ne = (
            self.waypoints[self.current_waypoint_index].x,
            self.waypoints[self.current_waypoint_index].y
        )
        
      
        
        delta_n = target_ne[0] - current_ne[0]
        delta_e = target_ne[1] - current_ne[1]
        distance_to_wp = np.hypot(delta_n, delta_e)
        desired_yaw = atan2(delta_e, delta_n)

        delta_goal_n = goal_ne.x - current_ne[0]
        delta_goal_e = goal_ne.y - current_ne[1]
        distance_to_goal = np.hypot(delta_goal_n, delta_goal_e)

        current_yaw = msg.orientation.yaw
        yaw_error = np.degrees(
            np.arctan2(np.sin(desired_yaw - current_yaw), np.cos(desired_yaw - current_yaw))
        )

        cmd = Rel_error_joystick()
        #stop = True
        if distance_to_wp < self.stop_threshold:
            if not self.reached_current:
                rospy.loginfo("Reached waypoint {}".format(self.current_waypoint_index + 1))
                
                self.stop_vehicle()
                self.reached_current = True
                
                if self.current_waypoint_index == 0 and not self.mission_started:
                    self.mission_start_pub.publish(Bool(data=True))
                    rospy.loginfo("Mission started.")
                    self.mission_started = True
                    self.mission_start_time = time.time()
                    self.mission_duration_timer = rospy.Timer(rospy.Duration(1), self.publish_mission_time)
            else:
                self.current_waypoint_index += 1
                self.reached_current = False
        else:
            if distance_to_goal > 2:
                if abs(yaw_error) > self.yaw_threshold:
                    if abs(yaw_error) < 45:
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
        """
        if distance_to_wp < self.stop_threshold:
            if not self.reached_current:
                rospy.loginfo("Reached waypoint {}".format(self.current_waypoint_index + 1))
                self.stop_vehicle()
                self.reached_current = True

                if self.current_waypoint_index == 0 and not self.mission_started:
                    self.mission_start_pub.publish(Bool(data=True))
                    rospy.loginfo("Mission started.")
                    self.mission_started = True
                    self.mission_start_time = time.time()
                    self.mission_duration_timer = rospy.Timer(rospy.Duration(1), self.publish_mission_time)
            else:
                self.current_waypoint_index += 1
                self.reached_current = False
        else:
            if abs(yaw_error) > self.yaw_threshold:

                print(self.yaw_threshold)
                print('ciao')
                cmd.error_surge_speed = 0.0
                cmd.error_yaw = yaw_error
            else:
                print('print')
                cmd.error_surge_speed = min(distance_to_wp * 0.5, self.max_speed)
                cmd.error_yaw = 0.0

            self.pub.publish(cmd)

            rospy.loginfo_throttle(
                1,
                "WP {}/{} | Distance: {:.2f} m | Yaw Error: {:.2f}° | Surge: {:.2f} m/s".format(
                    self.current_waypoint_index + 1,
                    len(self.waypoints),
                    distance_to_wp,
                    abs(yaw_error),
                    cmd.error_surge_speed
                )
            )
            """

    def stop_vehicle(self):
        stop_msg = Rel_error_joystick()
        self.pub.publish(stop_msg)
        rospy.loginfo("Vehicle stopped.")

    def waypoint_cb(self, msg):
        if msg.received and not self.got_waypoints:
            self.waypoints = msg.waypoints
            self.got_waypoints = True
            rospy.loginfo("Received {} waypoints.".format(len(self.waypoints)))

    def publish_mission_time(self, event):
        if self.mission_start_time:
            elapsed_time = time.time() - self.mission_start_time
            mission_time_msg = Float32(data=elapsed_time)
            self.mission_time_pub.publish(mission_time_msg)
            rospy.loginfo("Mission time: {:.2f} seconds.".format(elapsed_time))

    
    def stop_cb(self, msg):
        if msg.data:
            if not self.mission_stopped:
                rospy.logwarn("Mission stop signal received.")
                self.mission_stopped = True
                self.stop_vehicle()

                if self.mission_start_time:
                    self.elapsed_mission_time = time.time() - self.mission_start_time

                if self.mission_duration_timer:
                    self.mission_duration_timer.shutdown()
                    self.mission_duration_timer = None

        else:
            if self.mission_stopped:
                rospy.loginfo("Mission resume signal received.")
                self.mission_stopped = False

                if self.mission_started and not self.mission_duration_timer:
                    self.mission_start_time = time.time() - self.elapsed_mission_time
                    self.mission_duration_timer = rospy.Timer(rospy.Duration(1), self.publish_mission_time)

    def handle_shutdown(self):
        rospy.logwarn("Shutting down: sending stop command.")
        self.stop_vehicle()



if __name__ == "__main__":
    try:
        WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
