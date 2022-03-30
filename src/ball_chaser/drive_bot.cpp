#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "factorybot/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(factorybot::DriveToTarget::Request &req, factorybot::DriveToTarget::Response &res)
{
    geometry_msgs::Twist msg;
    msg.linear.x=req.linear_x;
    msg.angular.z=req.angular_z;
    motor_command_publisher.publish(msg);
    res.msg_feedback="Velocity request received -- Linear_x:"+std::to_string(msg.linear.x)+" and Angular_z:"+std::to_string(msg.angular.z);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/AMR/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}