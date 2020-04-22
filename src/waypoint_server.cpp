#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "software_challenge/waypointAction.h"

class waypointAction {

    protected:

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<software_challenge::waypointAction> as_;

        std::string action_name_;

        software_challenge::waypointActionFeedback feedback_;
        software_challenge::waypointActionResult result_;

    public:

        waypointAction(std::string name) :
            as_(nh_, name, boost::bind(&waypointAction::executeCB, this, _1), false),
            action_name_(name) {

            as_.start();
        }

        ~waypointAction(void) {
        }

        void executeCB(const software_challenge::waypointGoalConstPtr &goal) {
            
        }

    
};

int main(int argc, char** argv) {
    return 0;
}