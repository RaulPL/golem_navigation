/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 24/03/2015
*/

#ifndef DOOR_CHECKER_H
#define DOOR_CHECKER_H

#include "yaml-cpp/yaml.h"
#include "jsoncpp/json/json.h"
#include "ros/ros.h"
#include "json_msgs/JsonSrv.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

using namespace std;

class DoorChecker
{
public:
    //Methods
    DoorChecker(string path);
    bool isOpen(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response& res);
private:
    //Attributes
    YAML::Node yaml_node;
    string door_file;
};

#endif // DOOR_CHECKER_H
