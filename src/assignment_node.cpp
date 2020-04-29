#include "ros/ros.h"
#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "software_challenge/distance.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

float movingX, movingY;

void movingPoseCB(const turtlesim::Pose::ConstPtr& msg) {
    movingX = msg->x;
    movingY = msg->y;
}

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

void turtleDist () {
	ros::NodeHandle n;

	ros::Subscriber movingPoseSub = n.subscribe<turtlesim::Pose>("/moving_turtle/pose", 1000, &movingPoseCB);
    ros::Publisher distancePub = n.advertise<software_challenge::distance>("distances", 1000);

	software_challenge::distance msg;

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		msg.x = abs(movingX - 5);
		msg.y = abs(movingY - 5);
		msg.euclidean = sqrt(pow(msg.x, 2) + pow(msg.y, 2));

		distancePub.publish(msg);
		ROS_INFO("XDist: %f, YDist: %f, TotalDist: %f", msg.x, msg.y, msg.euclidean);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "software_training_assignment_node");
	ros::NodeHandle n;
    
	kill("turtle1");
	spawn("stationary_turtle", 5, 5, 0);
	spawn("moving_turtle", 10, 5, 0);
	turtleDist();

	return 0;
}