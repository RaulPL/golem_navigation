#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import sys
import rospy
from json_msgs.srv import *
from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseWithCovariance,
                               Quaternion, Pose, Point)


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
        rospy.init_node('swapper_client_node', anonymous=True)
        dist = float(sys.argv[1])
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        # pose = PoseWithCovarianceStamped()
        # pose.pose.pose.position.x = dist
        # pose.pose.pose.orientation.w = 1.0
        # cov = []
        # for i in range(6):
        #     for j in range(6):
        #         if i == j:
        #             cov.append(0.25)
        #         else:
        #             cov.append(0.0)
        # pose.pose.covariance = cov
        p = PoseWithCovarianceStamped()
        msg = PoseWithCovariance()
        msg.pose = Pose(Point(-0.767, -0.953, 0.000),
                        Quaternion(0.000, 0.000, -0.0, 0.9999))
        msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        p.pose = msg
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        print(p)
        pub.publish(p)
        # pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = 'map'
        # print(pose)
        # pub.publish(pose)

    except rospy.ROSInterruptException:
        print('Exception')
