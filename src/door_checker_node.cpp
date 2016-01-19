/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 25/03/2015
*/


#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <golem_navigation/door_checker.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_checker_node");

    string door_file = ros::package::getPath("navigation_services");
    door_file = door_file + "/resources/door_params.yaml";  //configuration file of the average dist and aperture
    DoorChecker dc(door_file);

    ros::NodeHandle n;
    ros::ServiceServer ss = n.advertiseService("door_checker_service", &DoorChecker::isOpen, &dc);
    ROS_INFO("READY: Providing the door_checker_service");
    ros::spin();
    return 0;
}
