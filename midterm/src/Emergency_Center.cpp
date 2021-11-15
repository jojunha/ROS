#include "ros/ros.h"
#include "midterm/emergency.h"

void EmergencyCallback(const midterm::emergency::ConstPtr& msg)
{
  ROS_INFO_STREAM("Emergency route : " << int(msg->route) << ", Distance : " << int(msg->distance));
  ROS_INFO_STREAM("Emergency route : " << int(msg->route) << ", Distance : " << int(msg->distance));
  ROS_INFO_STREAM("Emergency route : " << int(msg->route) << ", Distance : " << int(msg->distance));
}
int main(int argc, char **argv){
    ros::init(argc, argv, "Emergency_Center");
    ros::NodeHandle nh;
    ros::Subscriber sub_emergency = nh.subscribe("emergency", 1000, EmergencyCallback);

    ROS_INFO("Emergency Center Ready!");
    ros::spin();

    return 0;
}
