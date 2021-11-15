#include "ros/ros.h"
#include "midterm/camera.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <midterm/routeAction.h>

void CameraCallback(const midterm::camera::ConstPtr& msg)
{
    ROS_INFO_STREAM("Received CAMERA DATA : " << int(msg->c_data));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Control_Center");
    ros::NodeHandle nh;

    ros::Subscriber sub_camera = nh.subscribe("camera_data", 1000, CameraCallback);

    actionlib::SimpleActionClient<midterm::routeAction> ac("selected_route", true);
    ROS_INFO("Waiting for Robot_Control to start.");
    ac.waitForServer();

    ROS_INFO("Robot_Control, sending goal.");
    midterm::routeGoal goal;
    goal.r_order = atoll(argv[1]);
    ac.sendGoal(goal);
    bool success = true;

    while(ros::ok()){
      bool finished_before_timeout = ac.waitForResult(ros::Duration(1.0));
      if (finished_before_timeout && success)
      {
        actionlib::SimpleClientGoalState state = ac.getState();

        ROS_INFO("Complete the Route!!!");
        ROS_INFO("Complete the Route!!!");
        ROS_INFO("Complete the Route!!!");

        success = false;
      }
      ros::spinOnce();
    }
    return 0;
}
