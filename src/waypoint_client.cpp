#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "software_challenge/waypointAction.h"

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_waypoint");

    actionlib::SimpleActionClient<software_challenge::waypointAction> ac("waypoint", true);

    ROS_INFO("Waiting for waypoint action server to start...");
    ac.waitForServer();

    ROS_INFO("Waypoint action server started.  Where do you want the moving turtle to go?");

    float x, y;
    std::cout << "X Coordinate:";
    std::cin >> x;
    std::cout << "Y Coordinate:";
    std::cin >> y;

    software_challenge::waypointGoal goal;
    goal.x = x;
    goal.y = y;

    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));

    if (finished_before_timeout) {
        ROS_INFO("Succesfully moved the turtle to x=%f, y=%f", x, y);
    }
    else {
        ROS_ERROR("Turtle did not move to x=%f, y=%f before the time out.", x, y);
    }

    return 0;
}

