#!/bin/bash
xterm -geometry 80x5+0+0   -e "/opt/ros/indigo/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 10s
xterm -geometry 80x5+0+130 -e "/opt/ros/indigo/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 10s
xterm -geometry 80x5+0+390 -e "/opt/ros/indigo/bin/roslaunch dnavigation soundplay_node.launch" &
sleep 5s
xterm -geometry 80x5+0+650 -e "/opt/ros/indigo/bin/roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/demulab/myprog/src/0inspection/script/robocup20150718.yaml" &
sleep 10s
xterm -geometry 80x5+0+780 -e "/opt/ros/indigo/bin/roslaunch turtlebot_rviz_launchers view_navigation.launch --screen" &
sleep 10s
xterm -geometry 80x5+0+910 -e "/opt/ros/indigo/bin/rosrun inspection inspection" &
#xterm -geometry=80x5+0+910 -e "rospeex_audiomonitor audio_monitor_epd"
