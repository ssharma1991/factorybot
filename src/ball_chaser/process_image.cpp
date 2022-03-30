#include "ros/ros.h"
#include "factorybot/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    factorybot::DriveToTarget srv;
    srv.request.linear_x=lin_x;
    srv.request.angular_z=ang_z;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool found_ball = false;
    int leftmost_white_px = 9999;
    int rightmost_white_px = 0;
    for (int i = 0; i < img.height * img.step; i+=3){
        int r = img.data[i];
        int g = img.data[i+1];
        int b = img.data[i+2];

        if (r==white_pixel && g==white_pixel && b==white_pixel){
            int px_col= i % img.step/3;
            if (px_col < leftmost_white_px)
                leftmost_white_px = px_col;
            if (px_col > rightmost_white_px)
                rightmost_white_px = px_col;
            found_ball = true;
        }
    }
    
    if (!found_ball){
        std::cout<<"Ball NOT located"<<std::endl;
        drive_robot(0, 0);
        return;
    }                        
    
    int col_middle = (leftmost_white_px + rightmost_white_px)/2;
    if (col_middle < img.width * 0.4) {
        std::cout<<"Ball located on LEFT"<<std::endl;
        float scale_fact=(img.width * 0.5 - col_middle)/(img.width * 0.5);
        drive_robot(0.3, scale_fact * 1);
    } else if (col_middle > img.width * 0.6) {
        std::cout<<"Ball located on RIGHT"<<std::endl;
        float scale_fact=(col_middle - img.width * 0.5)/(img.width * 0.5);
        drive_robot(0.3, scale_fact * -1);
    } else if (leftmost_white_px < (img.width * 0.4) && rightmost_white_px > (img.width * 0.6)) {
        std::cout<<"Ball located too CLOSE"<<std::endl;
        drive_robot(0, 0);
    } else {
        std::cout<<"Ball located at CENTER"<<std::endl;
        drive_robot(0.3, 0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<factorybot::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/AMR/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}