/**
 * @file offb_raw_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

 gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
 */
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include <boost/format.hpp>

#include <opencv2/imgcodecs.hpp> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "robocars_esp32cam_bridge.hpp"




void initSub () {
    esp32cam_image_sub = node->subscribe("/esp32cam/raw_image_jpeg", 2, esp32cam_msg_cb);
}

void initPub() {
    ros_image_pub = node->advertise<sensor_msgs::Image>("/esp32cam/ros_image", 10);
}

static boost::format file_format;


void esp32cam_msg_cb(const sensor_msgs::ImageConstPtr& image_msg) {
   
    static uint32_t seq=0;
    sensor_msgs::Image img_msg; // >> message to be sent
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header
    std::string jpgFilename;

    header.seq = seq++; // user defined counter
    header.stamp = ros::Time::now(); // time

    cv::Mat rawData( 1, image_msg->data.size(), CV_8UC1, const_cast<void*>(reinterpret_cast<const void*>(image_msg->data.data())));
    //cv::Mat rawData(120, 160, CV_8UC1, const_cast<uchar*>(&image_msg->data[0]));

    /*
    jpgFilename = (file_format % "/d2-tmpfs/data/esp32cam/" % seq).str();
    std::ofstream myFile (jpgFilename, std::ios::out | std::ios::binary);
    std::ostream_iterator<char> output_iterator(myFile);
    std::copy(image_msg->data.begin(), image_msg->data.end(), output_iterator);
    myFile.close();
    */

    cv::Mat img = cv::imdecode( rawData , cv::IMREAD_COLOR );

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    
    ros_image_pub.publish(img_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocars_autopilot");
    
    file_format.parse("%s/esp32cam%08i.jpg");

    node = new ros::NodeHandle();

    initPub();
    initSub();

    ROS_INFO("esp32cam_bridge: Starting");

    // wait for FCU connection
    while(ros::ok()){
        ros::spin();
    }
}

