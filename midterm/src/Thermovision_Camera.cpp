#include "ros/ros.h"
#include "midterm/thermovision.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "Thermovision_Camera");
    ros::NodeHandle nh; 
    ros::Publisher pub_thermovision =
        nh.advertise<midterm::thermovision>("thermovision_data",1000);

    ros::Rate loop_rate(1);
    midterm::thermovision msg;

    int tem = 0;
    bool up = true;
    while (ros::ok()){

        msg.temperature = tem;
        ROS_INFO_STREAM("Thermovision Data : " << tem);
        pub_thermovision.publish(msg);

        ros::spinOnce;
        loop_rate.sleep();

        if (up){tem++;}
        else {tem--;}
        if (tem == 30 or tem == 0){up = !up;}
    }
    return 0;
}

