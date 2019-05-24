# ros_podo_connector(ROS2PODO)

A ros node, "ros_podo_connector", is developed for the users who are not familiar with PODO frameworks and about to run a mobile hubo(a real robot) provided by HUBO LAB.

Since the high level program(intelligence module) developers are usually more familiar with ROS environment, it is easier to move robot through a ros node. However, it is impossible to run Mobile hubo without HUBO LAB's private robot framework, PODO. Therefore, a node which connects ros environment and PODO frameworks is required.

"ros_podo_connector" subscribes topic "joint_state" and delivers it to PODO framework through TCP/IP communication.

</br>


## RUNNING joint_state_publish mode
1. RUN "ros_podo_connector"

   $ ros core
 
   $ rosrun ros_podo_connector ros_podo_connector

2. Launch MoveIT! configuration

   $ cd ~/catkin_ws && catkin_make
   $ source devel/setup.bash
   $ roslaunch mobile_hubo_omniWH_config demo.launch
   1) Move joints
   2) click plan and execute
   
   
3. RUN PODO with Qt editor
