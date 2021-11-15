#include "ros/ros.h"
#include "midterm/camera.h"
#include <cstdlib>

int main(int argc, char **argv){
    ros::init(argc, argv, "Camera_Sensor");
    ros::NodeHandle nh;
    ros::Publisher pub_camera =
        nh.advertise<midterm::camera>("camera_data",1000);

    ros::Rate loop_rate(1);
    midterm::camera msg;
    int camera_data;

    while (ros::ok()){
        camera_data = rand() % 30 + 1;
        msg.c_data = camera_data;

        ROS_INFO_STREAM("Camera Data : " << camera_data);
 
        pub_camera.publish(msg);

        ros::spinOnce;
        loop_rate.sleep();
    }
    return 0;
}

