#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <factorybot/pick_place_load.h>

class SubscribeAndPublish{
    private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber marker_sub;
    
    public:
    SubscribeAndPublish() {
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        marker_sub = n.subscribe("pick_place_load", 1, &SubscribeAndPublish::pick_place_callback, this);

        while (marker_pub.getNumSubscribers() < 1 && ros::ok()) {
            ROS_WARN("Please create a subscriber to the 'visualization_marker' topic");
            ros::Duration(1).sleep(); // sleep for 1 sec
        }
    }

    void pick_place_callback(const factorybot::pick_place_load& msg) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "payload";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        if (msg.action == factorybot::pick_place_load::PICK || msg.action == factorybot::pick_place_load::GO_PLACE) {
            marker.action = visualization_msgs::Marker::DELETE;
        } else if (msg.action == factorybot::pick_place_load::GO_PICK || msg.action == factorybot::pick_place_load::PLACE) {
            marker.action = visualization_msgs::Marker::ADD;
        }

        // Set the pose of the marker
        marker.pose.position.x = msg.x;
        marker.pose.position.y = msg.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(msg.theta/2);
        marker.pose.orientation.w = cos(msg.theta/2);

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
    ros::spin();
    return 0;
}
