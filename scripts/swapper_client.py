#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import sys
import rospy
from json_msgs.srv import *


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
        service_name = str(sys.argv[1])
        print(service_call(service_name))
    except rospy.ROSInterruptException:
        print('Exception')
