#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import sys
import rospy
from json_msgs.srv import *
from geometry_msgs.msg import PoseWithCovarianceStamped


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
                              queue_size=1)
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = dist
        pose.pose.pose.orientation.w = 1.0
        pub.publish(pose)

    except rospy.ROSInterruptException:
        print('Exception')
