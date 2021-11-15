#include "ros/ros.h"
#include "midterm/disinfection.h"

bool call(midterm::disinfection::Request &req, midterm::disinfection::Response &res)
{
    if (req.a == 1){
      ROS_INFO("Start Disinfection!");
      res.b = 1;}
    else if (req.a = 2) {
      ROS_INFO("Stop Disinfection!");
      res.b = 1;
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Disinfection");
  ros::NodeHandle nh;

  ros::ServiceServer brake = nh.advertiseService("disinfection", call);
  ROS_INFO("Ready for Disinfection.");
  ros::spin();

  return 0;
}
