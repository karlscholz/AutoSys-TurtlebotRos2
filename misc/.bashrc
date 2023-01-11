########## ROS2 TURTLEBOT3 ##########
export ROS_DOMAIN_ID=30 #TURTLEBOT3
export TURTLEBOT3_MODEL=waffle

cd ~/AutoSys-TurtlebotRos2/picproc_ws/
. install/setup.bash
cd -

alias rto='ros2 run turtlebot3_teleop teleop_keyboard'
alias picproc='ros2 run picproc picproc_node'

ifconfig | grep inet