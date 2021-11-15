#! /bin/bash
gnome-terminal -e "roscore" &
sleep 2
gnome-terminal -e "roslaunch midterm sensor.launch" &
sleep 2
gnome-terminal -e "rosrun midterm Robot_Control" &
sleep 2
gnome-terminal -e "rosrun midterm Emergency_Center" &
sleep 2
gnome-terminal -e "rosrun midterm Disinfection" &
sleep 
gnome-terminal -e "rosrun midterm Brake_System.py"
