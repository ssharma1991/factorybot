#include <math.h>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <factorybot/pick_place_load.h>
#define PI 3.14159265

class SubscribeAndPublish{
    private:
    // Define a client for to send goal requests to the move_base server through a SimpleActionClient
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    ros::NodeHandle n;
    ros::Publisher marker_pub;

    public:
    SubscribeAndPublish() {
        marker_pub = n.advertise<factorybot::pick_place_load>("pick_place_load", 1);
    }

    int createMission() {
        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac(n, "move_base", true);
        // Wait 5 sec for move_base action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // Move to Goal_1
        ROS_INFO("Sending Goal_1");
        if (moveToGoal (ac, "pick_mission", 2, 13.5, PI/2)) {
            ROS_INFO("The factorybot successfully moved to Goal_1");
        } else {
            ROS_INFO("The factorybot failed to move to Goal_1 for some reason");
            return 1;
        }

        ROS_INFO("Waiting for 5 seconds...");
        ros::Duration(5).sleep(); // sleep for 5 sec

        // Move to Goal_2
        ROS_INFO("Sending Goal_2");
        if (moveToGoal (ac, "place_mission", 13, -13.0, -PI/2)) {
            ROS_INFO("The factorybot successfully moved to Goal_2");
        } else {
            ROS_INFO("The factorybot failed to move to Goal_2 for some reason");
            return 1;
        }
        return 0;
    }

    bool moveToGoal (MoveBaseClient& ac, std::string mission, float x, float y, float theta) {
        // Order robot to move to goal
        publish_MoveBaseGoal(ac, x, y, theta);
        if (mission.compare("pick_mission")==0) {
            publish_pick_place_load_msg(factorybot::pick_place_load::GO_PICK, x, y, theta);
        } else if (mission.compare("place_mission")==0) {
            publish_pick_place_load_msg(factorybot::pick_place_load::GO_PLACE, x, y, theta);
        }

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (mission.compare("pick_mission")==0) {
            publish_pick_place_load_msg(factorybot::pick_place_load::PICK, x, y, theta);
        } else if (mission.compare("place_mission")==0) {
            publish_pick_place_load_msg(factorybot::pick_place_load::PLACE, x, y, theta);
        }
            return true;
        }
        return false;
    }

    void publish_MoveBaseGoal(MoveBaseClient& ac, float x, float y, float theta) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = sin(theta/2);
        goal.target_pose.pose.orientation.w = cos(theta/2);
        // Send the goal position and orientation for the robot to reach
        ac.sendGoal(goal);
    }

    void publish_pick_place_load_msg(int action, float x, float y, float theta) {
        factorybot::pick_place_load msg;
        msg.action = action;
        msg.x = x;
        msg.y = y;
        msg.theta = theta;
        marker_pub.publish(msg);
    }
};

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    SubscribeAndPublish pubsub;
    ros::Duration(5).sleep(); // sleep for 5 sec
    pubsub.createMission();
    return 0;
}