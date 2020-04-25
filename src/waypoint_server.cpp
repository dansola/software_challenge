#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "software_challenge/waypointAction.h"
#include "software_challenge/distance.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

float movingX, movingY;

void movingPoseCB(const turtlesim::Pose::ConstPtr& msg) {
    movingX = msg -> x;
    movingY = msg -> y;
}

typedef actionlib::SimpleActionServer<software_challenge::waypointAction> Server;

class waypointAction {

    protected:

        ros::NodeHandle nh_;
        Server as_;

        std::string action_name_;

        software_challenge::waypointFeedback feedback_;
        software_challenge::waypointResult result_;

    public:

        waypointAction (std::string name) :
            as_(nh_, name, boost::bind(&waypointAction::executeCB, this, _1), false),
            action_name_(name) {
                as_.start();
        }

        ~waypointAction (void) {
        }

        void executeCB (const software_challenge::waypointGoalConstPtr &goal) {

            ros::Subscriber movingPoseSub = nh_.subscribe("/moving_turtle/pose", 1000, movingPoseCB);
            ros::Publisher movingVelPub = nh_.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 1000);

            ROS_INFO("%s: Executing, moving moving_turtle to (%f %f).", action_name_.c_str(), goal->x, goal->y);

			bool success = true;

			ros::spinOnce();

			float relative_angle = atan(((goal->y - movingY))/(goal->x - movingX));
			float angular_speed = 0.1;
			float target_distance = sqrt(pow(movingX - goal->x, 2) + pow(movingY - goal->y, 2)); 
			float linear_speed = 0.5;

			ROS_INFO("Relative Angle  : %f", relative_angle);
			ROS_INFO("Angular Speed   : %f", angular_speed);
			ROS_INFO("Target Distance : %f", target_distance);

			geometry_msgs::Twist msg;

			msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = 0;
			msg.angular.z = angular_speed;


			float current_angle = 0;
			float current_distance = 0;

			ros::Rate r(100);

			ros::Time begin = ros::Time::now();
			r.sleep();
			movingVelPub.publish(msg);
			
			
			while (current_angle < relative_angle) {
				ROS_INFO("Current Angle: %f", current_angle);
				movingVelPub.publish(msg);
				
				float change = (ros::Time::now() - begin).toSec();
				current_angle = angular_speed * change;
				
				if (as_.isPreemptRequested() || !ros::ok()) {
			        ROS_INFO("%s: Preempted", action_name_.c_str());
			        as_.setPreempted();
			        success = false;
			        break;
			    }
            
				feedback_.distance = target_distance;
				as_.publishFeedback(feedback_);
				r.sleep();
			}

			msg.linear.x = linear_speed;
			msg.angular.z = 0;
			result_.time = (ros::Time::now() - begin).toSec();

			begin = ros::Time::now();

			while (current_distance < target_distance) {
				ROS_INFO("Current Distance: %f", current_distance);
				movingVelPub.publish(msg);
				float change = (ros::Time::now() - begin).toSec();
				current_distance = linear_speed * change;
				feedback_.distance = target_distance - current_distance;
				as_.publishFeedback(feedback_);				
				r.sleep();
			}

		
			result_.time += (ros::Time::now() - begin).toSec();

			if (success) {
				result_.time = (ros::Time::now() - begin).toSec();
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				//set action state to succeeded
				as_.setSucceeded(result_);
			}
		}  
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint");

    waypointAction waypoint(ros::this_node::getName());
    ros::spin();

    return 0;
}