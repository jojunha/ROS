#include "ros/ros.h"
#include "midterm/lidar.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "Lidar_Sensor");
    ros::NodeHandle nh;
    ros::Publisher pub_lidar =
        nh.advertise<midterm::lidar>("lidar_data",1000);

    ros::Rate loop_rate(1);
    midterm::lidar msg;
    int lidar_data;

    while (ros::ok()){
        lidar_data = rand() % 50 +1;
        msg.l_data = lidar_data;

        ROS_INFO_STREAM("Lidar Data : " << lidar_data);

        pub_lidar.publish(msg);

        ros::spinOnce;
        loop_rate.sleep();
    }
    return 0;
}

