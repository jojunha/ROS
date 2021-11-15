#!/usr/bin/env python

from midterm.srv import brake,brakeResponse
import rospy

def brakeCB(req):
    if(req.a == 1):
        rospy.loginfo("Do Brake!!")
    return brakeResponse(1)

def brake_server():
    rospy.init_node('Brake_System')
    s = rospy.Service('dobrake', brake, brakeCB)
    rospy.loginfo("Ready to Brake.")
    rospy.spin()

if __name__ == "__main__":
    brake_server()
