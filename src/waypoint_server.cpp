#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "software_challenge/waypointAction.h"
#include "software_challenge/distance.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

float movingX, movingY;

void movingPoseCB(const turtlesim::Pose::ConstPtr& msg) {
    movingX = msg->x;
    movingY = msg->y;
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

		float atanShift (float y2, float y1, float x2, float x1, float shift) {
			return shift + atan((y2 - y1)/ (x2 - x1));
		}

		float angleCalc (float x2, float y2, float x1, float y1) {
			if (x2 >= x1) {
				return atanShift(y2, y1, x2, x1, 0);
			}
			else if (y2 >= y1) {
				return atanShift(y2, y1, x2, x1, M_PI);
			}
			else {
				return atanShift(y2, y1, x2, x1, -M_PI);
			}
		}

		float calcDist (float x1, float x2, float y1, float y2) {
			return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
		}

		float setTime (ros::Time t_0) {
			ros::Time t_i = ros::Time::now();
			return (t_i - t_0).toSec();
		}

        void executeCB (const software_challenge::waypointGoalConstPtr &goal) {

            ros::Subscriber movingPoseSub = nh_.subscribe("/moving_turtle/pose", 1000, movingPoseCB);
            ros::Publisher movingVelPub = nh_.advertise<geometry_msgs::Twist>("moving_turtle/cmd_vel", 1000);

            ROS_INFO("%s: Executing, moving moving_turtle to (%f %f).", action_name_.c_str(), goal->x, goal->y);

			bool success = true;

			ros::spinOnce();

			float movingX = 10;
			float movingY = 5;

			float angle = angleCalc(goal->x, goal->x, movingX, movingY);
			float distance = calcDist(movingX, goal->x, movingY, goal->y);
			
			float dAngle = 0;
			float dDistance = 0;
			float dTheta_dt = 1;
			float dx_dt = 1;

			geometry_msgs::Twist msg;

			msg.linear.x = 0;
			msg.angular.z = dTheta_dt;

			ros::Rate loop_rate(100);

			loop_rate.sleep();
			movingVelPub.publish(msg);
			
			float dt;
			ros::Time t_i;
			ros::Time t_0 = ros::Time::now();

			while (dAngle < angle) {
				if (as_.isPreemptRequested() || !ros::ok()) {
			        ROS_INFO("%s: Preempted", action_name_.c_str());
			        as_.setPreempted();
			        success = false;
			        break;
			    }

				movingVelPub.publish(msg);
				
				dt = setTime(t_0);
				dAngle = dTheta_dt * dt;
				
				feedback_.distance = distance;
				as_.publishFeedback(feedback_);
				loop_rate.sleep();
			}

			msg.linear.x = dx_dt;
			msg.angular.z = 0;

			result_.time = setTime(t_0);
			t_0 = ros::Time::now();

			while (dDistance < distance) {
				if (as_.isPreemptRequested() || !ros::ok()) {
			        ROS_INFO("%s: Preempted", action_name_.c_str());
			        as_.setPreempted();
			        success = false;
			        break;
			    }

				movingVelPub.publish(msg);

				dt = setTime(t_0);
				dDistance = dx_dt * dt;

				feedback_.distance = distance - dDistance;
				as_.publishFeedback(feedback_);				
				loop_rate.sleep();
			}

			if (success) {
				result_.time = setTime(t_0);
				ROS_INFO("%s: Succeeded", action_name_.c_str());
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