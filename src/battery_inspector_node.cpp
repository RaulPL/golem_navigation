/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 25/03/2015
*/


#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <golem_navigation/battery_inspector.h>


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_inspector_node");
    BatteryInspector bi;
    ros::NodeHandle n;
    ros::ServiceServer ss = n.advertiseService("battery_inspector_service", &BatteryInspector::get_batterry_voltage, &bi);
    ROS_INFO("READY: Providing the battery_inspector_service");
    ros::spin();
    return 0;
}
