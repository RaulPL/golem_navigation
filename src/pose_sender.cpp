/*
 * Author: Raul Peralta Lozada
 * e-mail: raulpl25@gmail.com
 * date: 05/03/2015
*/

#include <ros/ros.h>
#include <math.h>
#include "golem_navigation/pose_sender.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>
#include "std_srvs/Empty.h"


PoseSender::PoseSender()
{

}

bool PoseSender::get_amcl_pose(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response &res){
	geometry_msgs::PoseWithCovarianceStampedConstPtr robot_pose;
	robot_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose");
	double roll, pitch, yaw;
	tf::Quaternion q(robot_pose->pose.pose.orientation.x, robot_pose->pose.pose.orientation.y, robot_pose->pose.pose.orientation.z, robot_pose->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	Json::FastWriter writer;
	Json::Value response;
	Json::Value position(Json::arrayValue);
	position.append(robot_pose->pose.pose.position.x);
	position.append(robot_pose->pose.pose.position.y);
	position.append(yaw);
	response["robot_pose"] = position;
	res.json = writer.write(response);
	return true;
}
