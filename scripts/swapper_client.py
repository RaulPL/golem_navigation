#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import sys
import rospy
from json_msgs.srv import *
from geometry_msgs.msg import (PoseWithCovarianceStamped, Quaternion, Point)


def service_call(service_name):
    rospy.wait_for_service(service_name)
    try:
        swapper_call = rospy.ServiceProxy(service_name, JsonSrv)
        res = swapper_call('{"Empty": "0"}')
        return res.json
    except rospy.ServiceException:
        rospy.loginfo('Error')


if __name__ == '__main__':
    try:
        # service_name = str(sys.argv[1])
        # print(service_call(service_name))
        rospy.init_node('swapper_client_node')
        dist = float(sys.argv[1])
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                              latch=True)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position = Point(-1.0, -1.0, 0.000)
        pose_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06]
        pose_msg.pose.covariance = covariance
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()
        print(pose_msg)
        rate = rospy.Rate(10)
        for i in range(100):
            pose_msg.header.stamp = rospy.Time.now()
            pub.publish(pose_msg)
            rate.sleep()
            # c_pose = rospy.wait_for_message('/amcl_pose',
            #                                 PoseWithCovarianceStamped)
            # if pose_msg.pose.pose.position.x == c_pose.pose.pose.position.x:
            #     if pose_msg.pose.pose.position.y == c_pose.pose.pose.position.y:
            #         break

    except rospy.ROSInterruptException:
        print('Exception')
