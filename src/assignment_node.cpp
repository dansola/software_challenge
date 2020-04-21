#include "ros/ros.h"
#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"

void kill (std::string turtle_name) {
	ros::NodeHandle n;
	ros::ServiceClient kill_client = n.serviceClient<turtlesim::Kill>("kill");
	turtlesim::Kill kill_srv;
	kill_srv.request.name = turtle_name;
	if (kill_client.call(kill_srv)) {
        ROS_INFO("Successfully killed %s.", turtle_name.c_str());
	}
	else {
		ROS_ERROR("Error: Failed to kill %s.", turtle_name.c_str());
	}
}


void spawn (std::string turtle_name, float x, float y, float theta) {
	ros::NodeHandle n;
	ros::ServiceClient spawn_client = n.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn spawn_srv;
	spawn_srv.request.name = turtle_name;
	spawn_srv.request.x = x;
	spawn_srv.request.y = y;
	spawn_srv.request.theta = theta;
	if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Successfully spawned %s at x=%f y=%f", turtle_name.c_str(), x, y);
	}
	else {
		ROS_ERROR("Error: Failed to spawn %s.", turtle_name.c_str());
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "software_training_assignment_node");
	ros::NodeHandle n;
    
	kill("turtle1");
	spawn("stationary_turtle", 5, 5, 0);
	spawn("moving_turtle", 10, 5, 0);

	return 0;
}