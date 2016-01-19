/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 05/03/2015
*/

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <golem_navigation/pose_sender.h>

using namespace std;

int main(int argc, char** argv)
{
    //Simple service that sends the current 
    //pose of the robot in a json message.
    ros::init(argc, argv, "pose_sender_node");
    PoseSender ps;

    ros::NodeHandle nh;
    ros::ServiceServer ss = nh.advertiseService("pose_sender_service", &PoseSender::get_amcl_pose, &ps);
    ROS_INFO("READY: Providing the pose_sender_service");
    ros::spin();
    return 0;
}

