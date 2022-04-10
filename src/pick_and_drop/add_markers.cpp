#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <factorybot/pick_place_load.h>

class SubscribeAndPublish{
    private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    
    public:
    SubscribeAndPublish() {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        ROS_INFO("PubSub setup complete");

        while (marker_pub.getNumSubscribers() < 1 && ros::ok()) {
            ROS_WARN("Please create a subscriber to the marker");
            ros::Duration(1).sleep(); // sleep for 1 sec
        }
    }

    void publish_cube_marker(int action, float x, float y, float theta) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "payload";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = action;

        // Set the pose of the marker
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(theta/2);
        marker.pose.orientation.w = cos(theta/2);

        // Set the scale of the marker
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_pub.publish(marker);
    }

};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    SubscribeAndPublish pubsub;

    ROS_INFO("Displaying Marker_1");
    pubsub.publish_cube_marker(visualization_msgs::Marker::ADD, 12.5, 7.0, 0);

    ROS_INFO("Waiting for 5 seconds ...");
    ros::Duration(5).sleep();

    ROS_INFO("Hiding Marker_1");
    pubsub.publish_cube_marker(visualization_msgs::Marker::DELETE, 12.5, 7.0, 0);

    ROS_INFO("Waiting for 5 seconds ...");
    ros::Duration(5).sleep();

    ROS_INFO("Displaying Marker_2");
    pubsub.publish_cube_marker(visualization_msgs::Marker::ADD, 13, -13.0, 0);
    ros::Duration(5).sleep();
    
    return 0;
}
