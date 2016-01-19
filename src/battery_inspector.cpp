/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 24/03/2015
*/


#include "golem_navigation/battery_inspector.h"
#include "std_msgs/Float64.h"


using namespace std;

BatteryInspector::BatteryInspector()
{

}

bool BatteryInspector::get_batterry_voltage(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response& res)
{
    std_msgs::Float64ConstPtr b_p = ros::topic::waitForMessage<std_msgs::Float64>("/RosAria/battery_voltage");
    Json::FastWriter writer;
    Json::Value response;
    response["battery_voltage"] = b_p->data;
	res.json = writer.write(response);
    return true;
}
