#include <ros/ros.h>
#include <stdio.h>


ros::NodeHandle * node;
ros::Subscriber esp32cam_image_sub;
ros::Publisher ros_image_pub;


void esp32cam_msg_cb(const sensor_msgs::ImageConstPtr& image_msg);

