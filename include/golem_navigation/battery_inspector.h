/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 24/03/2015
*/


#ifndef BATTERY_INSPECTOR_H
#define BATTERY_INSPECTOR_H

#include "jsoncpp/json/json.h"
#include "ros/ros.h"
#include "json_msgs/JsonSrv.h"


using namespace std;

class BatteryInspector
{
public:
    //Methods
    BatteryInspector();
    bool get_batterry_voltage(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response& res);
};

#endif // BATTERY_INSPECTOR_H
