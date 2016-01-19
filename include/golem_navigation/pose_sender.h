/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 05/03/2015
*/

#ifndef POSE_SENDER_H
#define POSE_SENDER_H

#include <string>
#include "jsoncpp/json/json.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json_msgs/JsonSrv.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"



using namespace std;

class PoseSender
{
public:
    //Methods
    PoseSender();
    bool get_amcl_pose(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response& res);

};

#endif // POSE_SENDER_H
