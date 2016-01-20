#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import rospy
import actionlib
import json
from math import cos, sin, pi

from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from json_msgs.msg import JsonMsg
from json_msgs.srv import *
from tf.transformations import quaternion_from_euler


class Follower(object):
    def __init__(self):
        rospy.init_node('Follower', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('Follower node running ...')

        self.person_detected = False
        self.follow = False
        self.distance = 100

        self.pose_sub = rospy.Subscriber('/followed_person_msg', JsonMsg,
                                         self.person_callback, queue_size=1)

        self.marker_pub = rospy.Publisher('/person_marker',
                                          Marker, queue_size=1)

        # Marker definition
        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        self.marker.header.stamp = rospy.Time.now()
        self.marker.id = 0
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.75
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.scale.z = 1.5
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.2
        self.marker.color.a = 1.0

        self.start_follow_service = rospy.Service('start_follow_person',
                                            JsonSrv, self.start_follow_person)

        self.stop_follow_service = rospy.Service('stop_follow_person',
                                            JsonSrv, self.stop_follow_person)

        self.move_base_ac = actionlib.SimpleActionClient('move_base',
                                                         MoveBaseAction)
        rospy.loginfo('Waiting for move_base action server ...')
        self.move_base_ac.wait_for_server(rospy.Duration(5))
        rospy.loginfo('Move_base action server running ...')
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = '/base_link'

    def start_follow_person(self, req):
        json_dict = json.loads(req.json)
        self.distance = json_dict['distance']
        self.follow = True
        rospy.loginfo('Following status: {0}'.format(self.follow))
        return JsonSrvResponse("{'status':'following'}")

    def stop_follow_person(self, req):
        self.follow = False
        self.move_base_ac.cancel_goal()
        rospy.loginfo('Following status: {0}'.format(self.follow))
        return JsonSrvResponse("{'status':'not_following'}")

    def person_callback(self, msg):
        json_dict = json.loads(msg.json)
        if json_dict['text'] == ['null'] or json_dict['text'] == '[0,0]':
            if self.person_detected:
                rospy.loginfo('Not following')
            self.person_detected = False
            self.move_base_ac.cancel_goal()
        else:
            # Update the information of the marker and goal
            self.person_detected = True
            d, a = json.loads(json_dict['text'])  # Get dist and angle
            if not self.person_detected:
                rospy.loginfo('Following person at({0}, {1})'.format(d, a))
            target_d = d - self.distance  # Remove distance
            if target_d < 0:
                target_d = 0
            a *= pi/180  # Change to radians
            marker_coord = Point((d/100.0)*cos(abs(a)), 0.0, 0.75)
            target_coord = Point((target_d/100.0)*cos(abs(a)), 0.0, 0.0)

            if a > 0.0:
                marker_coord.y = -(d/100.0)*sin(abs(a))
                target_coord.y = -(target_d/100.0)*sin(abs(a))
            else:
                marker_coord.y = (d/100.0)*sin(abs(a))
                target_coord.y = (target_d/100.0)*sin(abs(a))

            # Update marker position
            self.marker.pose.position = marker_coord

            # Update target pose
            self.goal.target_pose.pose.position = target_coord
            q = quaternion_from_euler(0.0, 0.0, -1*a)
            self.goal.target_pose.pose.orientation.x = q[0]
            self.goal.target_pose.pose.orientation.y = q[1]
            self.goal.target_pose.pose.orientation.z = q[2]
            self.goal.target_pose.pose.orientation.w = q[3]

    def run(self):
        while not rospy.is_shutdown():
            if self.person_detected and self.follow:
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.move_base_ac.send_goal(self.goal)
                # Maybe we can remove this line
                goal_finished = self.move_base_ac.wait_for_result(
                    rospy.Duration.from_sec(0.5)
                )
            self.marker_pub.publish(self.marker)

    def shutdown(self):
        rospy.loginfo('Stopping kinect_to_pose converter node ...')
        self.move_base_ac.cancel_goal()


if __name__ == "__main__":
    follower = Follower()
    follower.run()
    rospy.spin()
