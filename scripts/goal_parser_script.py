#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import os
import yaml
import json
import rospy
import actionlib
import time

from math import pi
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from json_msgs.srv import *


class GoalParserException(Exception):
    pass


class GoalParser(object):
    def __init__(self, places_path, vel_params):
        rospy.loginfo('Waiting for move_base services ...')
        rospy.wait_for_service('/move_base/clear_costmaps')
        if not (os.path.exists(places_path) and os.path.exists(vel_params)):
            raise GoalParserException('Invalid paths for the parameters.')
        with open(places_path, 'r') as f:
            self.places = yaml.safe_load(f)
        with open(vel_params, 'r') as f:
            self.velocities = yaml.safe_load(f)

        rospy.init_node('goal_parser_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.navigate_srv = rospy.Service('/navigate_service',
                                          JsonSrv, self.navigate)
        self.clear_map_sc = rospy.ServiceProxy('move_base/clear_costmaps',
                                               Empty)
        self.velocity_pub = rospy.Publisher('/RosAria/cmd_vel', Twist,
                                            queue_size=10)

        self.move_base_ac = actionlib.SimpleActionClient('move_base',
                                                         MoveBaseAction)
        self.move_goal = MoveBaseGoal()
        rospy.loginfo('GoalParser node running ...')

    def navigate(self, req):
        try:
            self.clear_map_sc()
        except rospy.ServiceException:
            rospy.loginfo('Failed to call ~clear_costmaps service')
        try:
            json_dict = json.loads(req.json)
        except json.JSONDecoder:
            rospy.loginfo('Invalid JSON string.')
            return JsonSrvResponse('{"status" : "navigation_error"}')
        command, value = json_dict.popitem()
        goal_pose = PoseStamped()
        print(value)
        if command == 'navigate':
            # Move the mobile base to a desired location in the map.
            # It can be a labeled place or a desired coordinate.
            self.move_goal.target_pose.header.frame_id = '/map'
            if isinstance(value, unicode) or isinstance(value, str):
                # Extract the name of the place
                try:
                    coord = self.places[value]
                    print(coord)
                except KeyError:
                    rospy.loginfo('Place name was not defined.')
                    return JsonSrvResponse('{"status" : "navigation_error"}')
                goal_pose.pose.position.x = coord[0]
                goal_pose.pose.position.y = coord[1]
                # goal_pose.pose.position.z = coord[2]
                goal_pose.pose.orientation.x = coord[3]
                goal_pose.pose.orientation.y = coord[4]
                goal_pose.pose.orientation.z = coord[5]
                goal_pose.pose.orientation.w = coord[6]
            else:
                # Read the pose
                goal_pose.pose.position.x = float(value['x'])
                goal_pose.pose.position.y = float(value['y'])
                angle = (float(value['angle']) * pi) / 180.0  # in radians
                q = quaternion_from_euler(0, 0, angle)
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]
                print(goal_pose)
            self.move_goal.target_pose.pose = goal_pose.pose
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo('Sending goal ...')
            self.move_base_ac.send_goal(self.move_goal)
            state = self.move_base_ac.wait_for_result(
                rospy.Duration.from_sec(40))
            if state:
                rospy.loginfo('navigate OK')
                return JsonSrvResponse('{"status" : "ok"}')
            rospy.loginfo('navigate FAILED')
            return JsonSrvResponse('{"status":"navigation_error"}')
        elif command == 'advance':
            self.move_goal.target_pose.header.frame_id = '/base_link'
            distance = float(value) if (isinstance(value, str) or
                                        isinstance(value, unicode)) else value
            goal_pose.pose.position.x = distance
            goal_pose.pose.orientation.w = 1
            self.move_goal.target_pose.pose = goal_pose.pose
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            print(self.move_goal)
            rospy.loginfo('Sending goal ...')
            self.move_base_ac.send_goal(self.move_goal)
            state = self.move_base_ac.wait_for_result(
                rospy.Duration.from_sec(40))
            if state:
                rospy.loginfo('advance OK')
                return JsonSrvResponse('{"status":"ok"}')
            rospy.loginfo('advance FAILED')
            return JsonSrvResponse('{"status":"navigation_error"}')
        elif command == 'turn':
            self.move_goal.target_pose.header.frame_id = '/base_link'
            angle = float(value) if (isinstance(value, str) or
                                     isinstance(value, unicode)) else value
            angle = (-angle * pi) / 180.0  # angle in radians
            q = quaternion_from_euler(0, 0, angle)
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
            self.move_goal.target_pose.pose = goal_pose.pose
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            rospy.loginfo('Sending goal ...')
            print(self.move_goal)
            self.move_base_ac.send_goal(self.move_goal)
            state = self.move_base_ac.wait_for_result(
                rospy.Duration.from_sec(40))
            if state:
                rospy.loginfo('turn OK')
                return JsonSrvResponse('{"status":"ok"}')
            rospy.loginfo('turn FAILED')
            return JsonSrvResponse('{"status":"navigation_error"}')
        elif command == 'advance_fine' or 'turn_fine':
            try:
                robot_pose = rospy.wait_for_message('/amcl_pose',
                                                    PoseWithCovarianceStamped)
                robot_odom = rospy.wait_for_message('/RosAria/pose',
                                                    Odometry, 1)
            except rospy.ROSException:
                rospy.loginfo('Unable to get current pose or odometry.')
                return JsonSrvResponse('{"status":"navigation_error"}')
            rate = rospy.Rate(10)
            twist = Twist()
            if command == 'advance_fine':
                tolerance_d = 0.04
                distance = float(value) if (isinstance(value, str) or
                                            isinstance(value, unicode)
                                            ) else value
                starting_amcl_x = robot_pose.pose.pose.position.x
                starting_aria_x = robot_odom.pose.pose.position.x
                # Determine the direction of the movement
                if starting_amcl_x < (starting_amcl_x + distance):
                    twist.linear.x = self.velocities['lin_vel']
                else:
                    twist.linear.x = -self.velocities['lin_vel']
                self.velocity_pub.publish(twist)
                t = time.time()
                while True:
                    if (time.time() - t) < 60:
                        try:
                            robot_odom = rospy.wait_for_message(
                                '/RosAria/pose', Odometry)
                        except rospy.ROSException:
                            rospy.loginfo('Unable to get current pose '
                                          'from /RosAria/pose.')
                            twist.linear.x = 0
                            self.velocity_pub.publish(twist)
                            return JsonSrvResponse(
                                '{"status":"navigation_error"}')
                        # reference frame will be the curr pose of the robot
                        curr_distance = abs(
                            robot_odom.pose.pose.position.x - starting_aria_x)

                        if abs(distance - curr_distance) < tolerance_d:
                            twist.linear.x = 0
                            self.velocity_pub.publish(twist)
                            rospy.loginfo('advance_fine OK')
                            return JsonSrvResponse('{"status":"ok"}')
                            # break
                        rate.sleep()
                    else:
                        twist.linear.x = 0
                        self.velocity_pub.publish(twist)
                        rospy.loginfo('advance_fine FAILED')
                        return JsonSrvResponse('{"status":"navigation_error"}')
            else:
                # 'turn_fine'
                tolerance_a = (5 * pi)/180.0
                angle = float(value) if (isinstance(value, str) or
                                         isinstance(value, unicode)
                                         ) else value
                angle = -(angle * pi)/180.0  # Change to radians and sign
                starting_amcl_a = euler_from_quaternion(
                    [robot_pose.pose.pose.orientation.x,
                     robot_pose.pose.pose.orientation.y,
                     robot_pose.pose.pose.orientation.z,
                     robot_pose.pose.pose.orientation.w]
                )
                starting_aria_a = euler_from_quaternion(
                    [robot_odom.pose.pose.orientation.x,
                     robot_odom.pose.pose.orientation.y,
                     robot_odom.pose.pose.orientation.z,
                     robot_odom.pose.pose.orientation.w]
                )
                # Determine the direction of the movement (a[2] -> yaw)
                if starting_amcl_a[2] < (starting_amcl_a[2] + angle):
                    twist.angular.z = self.velocities['ang_vel']
                else:
                    twist.angular.z = -self.velocities['ang_vel']
                self.velocity_pub.publish(twist)
                t = time.time()
                while True:
                    if (time.time() - t) < 60:
                        try:
                            robot_odom = rospy.wait_for_message(
                                '/RosAria/pose', Odometry)
                        except rospy.ROSException:
                            rospy.loginfo('Unable to get current pose '
                                          'from /RosAria/pose.')
                            twist.angular.z = 0
                            self.velocity_pub.publish(twist)
                            return JsonSrvResponse(
                                '{"status":"navigation_error"}')
                        current_a = euler_from_quaternion(
                            [robot_odom.pose.pose.orientation.x,
                             robot_odom.pose.pose.orientation.y,
                             robot_odom.pose.pose.orientation.z,
                             robot_odom.pose.pose.orientation.w]
                        )
                        # reference frame will be the curr pose of the robot
                        curr_distance = 0
                        if (starting_aria_a[2] > 0 and (starting_aria_a[2] + angle < pi)) or (starting_aria_a[2] < 0 and (starting_aria_a[2] + angle > -pi)):
                            curr_distance = abs(
                                    current_a[2] - starting_aria_a[2])
                        else:
                            if starting_aria_a[2] > 0 and current_a[2] > 0:
                                curr_distance = abs(
                                        current_a[2] - starting_aria_a[2])
                            elif starting_aria_a[2] > 0 > current_a[2]:
                                curr_distance = abs(
                                        2 * pi + current_a[2] - starting_aria_a[2])
                            elif starting_aria_a[2] < 0 and current_a[2] < 0:
                                curr_distance = abs(
                                        current_a[2] - starting_aria_a[2])
                            # (starting_aria_a[2] < 0 and current_a[2] > 0)
                            else:
                                curr_distance = abs(
                                        -((2 * pi) - current_a[2]) - starting_aria_a[2])

                        if abs(angle - curr_distance) < tolerance_a:
                            twist.angular.z = 0
                            self.velocity_pub.publish(twist)
                            # Send response
                            rospy.loginfo('turn_fine OK')
                            return JsonSrvResponse('{"status":"ok"}')
                        rate.sleep()
                    else:
                        twist.angular.z = 0
                        self.velocity_pub.publish(twist)
                        rospy.loginfo('turn_fine FAILED')
                        return JsonSrvResponse('{"status":"navigation_error"}')
        else:
            rospy.loginfo('Invalid navigation command!')
            return JsonSrvResponse('{"status" : "navigation_error"}')

    @staticmethod
    def run():
        while not rospy.is_shutdown():
            pass

    def shutdown(self):
        self.move_base_ac.cancel_all_goals()
        rospy.loginfo('Stopping GoalParser node ...')


if __name__ == '__main__':
    places_path = rospy.get_param('/goal_parser_node/places_path')
    vel_params = rospy.get_param('/goal_parser_node/vel_params')
    gp = GoalParser(places_path, vel_params)
    gp.run()
