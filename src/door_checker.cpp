#include "golem_navigation/door_checker.h"

using namespace std;

DoorChecker::DoorChecker(string path)
{
    yaml_node = YAML::LoadFile(path);
    door_file = path;
}

bool DoorChecker::isOpen(json_msgs::JsonSrv::Request& req, json_msgs::JsonSrv::Response& res)
{
    yaml_node = YAML::LoadFile(door_file);
	double min_dist = yaml_node["door_distance"].as<double>();
    int aperture_rad = yaml_node["aperture_rad"].as<int>();
	sensor_msgs::LaserScan::ConstPtr laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan_filtered");

	double average_dist = 0;
	int contador = 0;
	for (unsigned int i = (90 - aperture_rad); i < (90 + aperture_rad); i++){
	    if (laser_scan->ranges[i] == laser_scan->ranges[i]){
		    average_dist = average_dist + laser_scan->ranges[i];
		    contador = contador + 1;
		}
	}
	average_dist /= contador;
    cout<<"min_distance: "<<min_dist<<", "<<"distance: "<<average_dist<<endl;
    Json::FastWriter writer;
    Json::Value response;
    if (average_dist > min_dist){
        response["door_status"] = "open";
		res.json = writer.write(response);
        cout<<"Open"<<endl;
    }
    else{
		response["door_status"] = "closed";
		res.json = writer.write(response);	
        cout<<"Closed"<<endl;
	}
	return true;
}
