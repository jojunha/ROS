#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <midterm/routeAction.h>
#include "midterm/lidar.h"
#include "midterm/thermovision.h"
#include "midterm/emergency.h"
#include "midterm/brake.h"
#include "midterm/disinfection.h"
#include <cstdlib>

class Robot_Control
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<midterm::routeAction> as_;

  midterm::routeFeedback feedback_;
  midterm::routeResult result_;
  midterm::emergency e_msg;

  int waypoint_number = 0;
  int distance = 0;
  bool control_state = false;

  ros::Subscriber sub_lidar;
  ros::Subscriber sub_thermovision;
  ros::Publisher pub_emergency;

public:

  Robot_Control(std::string name) :
    as_(nh_, name, boost::bind(&Robot_Control::WaypointCB, this, _1), false)
  {
    sub_lidar = nh_.subscribe("lidar_data", 1000, &Robot_Control::lidarCallback, this);
    sub_thermovision = nh_.subscribe("thermovision_data", 1000, &Robot_Control::thermovisionCallback, this);

    pub_emergency = nh_.advertise<midterm::emergency>("emergency",1000);

    as_.start();
  }

  ~Robot_Control(void){}

  void lidarCallback(const midterm::lidar::ConstPtr& msg)
  {
    if(control_state){
      ROS_INFO_STREAM("Received Lidar Data : " << int(msg->l_data));
    }
  }

  void thermovisionCallback(const midterm::thermovision::ConstPtr& msg)
  {
    if(control_state){
      ROS_INFO_STREAM("Received Thermovision Data : " << int(msg->temperature));
      if(msg->temperature % 15 == 0){
        e_msg.route = waypoint_number;
        e_msg.distance = distance;
        pub_emergency.publish(e_msg);
      }
    }
  }

  void WaypointCB(const midterm::routeGoalConstPtr &goal)
  {
    control_state = true;
    ros::Rate loop_rate(1);

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Preempted!");
      as_.setPreempted();
    }

    midterm::disinfection d_srv;
    ros::ServiceClient client_disinfection = nh_.serviceClient<midterm::disinfection>("disinfection");

    d_srv.request.a = 1;
    if(client_disinfection.call(d_srv)){
      ROS_INFO("Start Disinfection!!!");
    }

    waypoint_number = int(goal->r_order);

    ROS_INFO_STREAM("Start the Route " << waypoint_number);
    ROS_INFO_STREAM("Start the Route " << waypoint_number);

    bool up = true;

    while(ros::ok()){
      switch (waypoint_number) {
      case 1 :
        if (up){distance++;}
        else {distance--;}
        if (distance == 10){
          up = !up;
          ROS_INFO("Halfway Point of Route 1!");}
        break;
      case 2 :
        if (up){distance++;}
        else {distance--;}
        if (distance == 20){
          up = !up;
          ROS_INFO("Halfway Point of Route 2!");}
        break;
      case 3 :
        if (up){distance++;}
        else {distance--;}
        if (distance == 30){
          up = !up;
          ROS_INFO("Halfway Point of Route 3!");}
        break;
      }

      ROS_INFO_STREAM("Distance : " << distance);
      feedback_.r_feedback = distance;
      as_.publishFeedback(feedback_);

      ros::spinOnce();
      loop_rate.sleep();
      if(!up && distance == 1){
        ros::ServiceClient client_brake = nh_.serviceClient<midterm::brake>("dobrake");
        midterm::brake b_srv;
        b_srv.request.a = 1;
        if(client_brake.call(b_srv)){
            ROS_INFO("Brake Compelete!");
            ROS_INFO("Brake Compelete!");
            ROS_INFO("Brake Compelete!");
        }
      }
      if(distance == 0){
        ROS_INFO_STREAM("Complete the Route " << waypoint_number);
        ROS_INFO_STREAM("Complete the Route " << waypoint_number);
        ROS_INFO_STREAM("Complete the Route " << waypoint_number);

        midterm::disinfection d_srv;
        ros::ServiceClient client_disinfection = nh_.serviceClient<midterm::disinfection>("disinfection");

        d_srv.request.a = 2;
        if(client_disinfection.call(d_srv)){
          ROS_INFO("Stop Disinfection!!!");
        }

        result_.r_result = 1;
        as_.setSucceeded(result_);
        control_state = false;
        break;}
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Robot_Control");
  ros::NodeHandle nh;

  Robot_Control robot("selected_route");
  ROS_INFO("Robot Control Ready!");

  ros::spin();

  return 0;
}
