#!/usr/bin/env python

# Created by Raul Peralta-Lozada

import rospy
import os
import json

from roslaunch import rlutil, parent, configure_logging
from geometry_msgs.msg import PoseWithCovarianceStamped
from json_msgs.srv import *


class Swapper(object):
    def __init__(self, amcl_path, gmapping_path, init_method):
        if not (os.path.exists(amcl_path) and os.path.exists(gmapping_path)):
            raise NameError('Launch files do not exist.')
        self.method_paths = {'amcl': amcl_path, 'gmapping': gmapping_path}
        rospy.init_node('swapper_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        try:
            # Run the corresponding launch file
            self.launch_process = Swapper.launch_file(
                    self.method_paths[init_method])
        except KeyError:
            raise NameError('Invalid localization method name. Available '
                            'names are "amcl" and "gmapping".')
        self.current_method = init_method

        # Define the services
        self.name_service = rospy.Service('get_method_name', JsonSrv,
                                          self.get_method_name)
        self.amcl_service = rospy.Service('start_amcl', JsonSrv,
                                          self.start_amcl)
        self.gmapping_service = rospy.Service('start_gmapping', JsonSrv,
                                              self.start_gmapping)
        self.pose_pub = rospy.Publisher('/initialpose',
                                        PoseWithCovarianceStamped,
                                        queue_size=1, latch=True)

        self.amcl_pose = rospy.wait_for_message('/amcl_pose',
                                                PoseWithCovarianceStamped)

    def get_method_name(self, req):
        json_str = json.dumps(dict(method=self.current_method))
        return JsonSrvResponse(json_str)

    def start_amcl(self, req):
        self.launch_process.shutdown()
        self.launch_process = Swapper.launch_file(self.method_paths['amcl'])
        self.current_method = 'amcl'
        # load the last stored pose
        rospy.loginfo(
                'Setting AMCL pose: {0}, {1}'.format(
                        self.amcl_pose.pose.pose.position,
                        self.amcl_pose.pose.pose.orientation)
        )
        json_str = json.dumps(dict(method=self.current_method))
        rospy.wait_for_service('/global_localization')
        # TODO: Find a better way to initialize the pose in amcl
        rate = rospy.Rate(10)
        for i in range(10):
            self.amcl_pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.amcl_pose)
            rate.sleep()
        return JsonSrvResponse(json_str)

    def start_gmapping(self, req):
        # Save the current pose
        self.amcl_pose = rospy.wait_for_message('/amcl_pose',
                                                PoseWithCovarianceStamped)
        self.launch_process.shutdown()
        rospy.loginfo(
                'Storing AMCL pose: {0}, {1}'.format(
                        self.amcl_pose.pose.pose.position,
                        self.amcl_pose.pose.pose.orientation)
        )
        self.launch_process = Swapper.launch_file(
                self.method_paths['gmapping'])
        self.current_method = 'gmapping'
        json_str = json.dumps(dict(method=self.current_method))
        return JsonSrvResponse(json_str)

    @staticmethod
    def launch_file(file_path):
        uuid = rlutil.get_or_generate_uuid(None, False)
        configure_logging(uuid=uuid)
        launch_process = parent.ROSLaunchParent(uuid, [file_path])
        launch_process.start()
        launch_process.spin_once()
        return launch_process

    @staticmethod
    def run():
        while not rospy.is_shutdown():
            pass

    def shutdown(self):
        self.launch_process.shutdown()
        rospy.loginfo('Stopping Swapper node ...')

if __name__ == '__main__':
    amcl_path = rospy.get_param('/swapper_node/amcl_path')
    gmapping_path = rospy.get_param('/swapper_node/gmapping_path')
    swapper = Swapper(amcl_path, gmapping_path, 'amcl')
    swapper.run()
