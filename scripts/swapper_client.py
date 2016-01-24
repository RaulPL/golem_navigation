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


class TestInit(object):
    def __init__(self):
        rospy.init_node('swapper_client_node')
        self.FLAG = True
        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.pose.pose.position = Point(-1.0, -1.0, 0.000)
        self.pose_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06]
        self.pose_msg.pose.covariance = covariance
        self.pose_msg.header.frame_id = 'map'
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,
                                   latch=True, queue_size=50)
        self.sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped,
                                    self.initial_pose_cb, queue_size=1)

    def initial_pose_cb(self, data):
        print(self.pose_msg)
        self.FLAG = False

    def run(self):
        while self.FLAG:
            self.pose_msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.pose_msg)


if __name__ == '__main__':
    try:
        # service_name = str(sys.argv[1])
        # print(service_call(service_name))

        # dist = float(sys.argv[1])
        test = TestInit()
        test.run()

    except rospy.ROSInterruptException:
        print('Exception')
