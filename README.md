# ros_podo_connector(ROS2PODO)

A ros node, "ros_podo_connector", is developed for the users who are not familiar with PODO frameworks and about to run mobile hubo provided by HUBO LAB.

Since the high level program(intelligence module) developers are usually more familiar with ROS environment, it is easier to move robot through a ros node. However, it is impossible to run Mobile hubo without HUBO LAB's private robot framework, PODO, so a node which connects ros environment and PODO frameworks is required.

"ros_podo_connector" subscribes topic "joint_state", a desired joint angles, and deliver it to PODO framework through TCP/IP communication.

To verfy this ros node, followings are required
* ros project "ros_podo_connector"
* PODO
* mobile hubo moveit configuration (This can be replaced any ros node which publish joint_state) 


</br></br>

1. RUN "ros_podo_connector"

   $ ros core
 
   $ rosrun ros_podo_connector ros_podo_connector

2. Launch MoveIT! configuration

   $ roslaunch mobile_hubo_omniWH_config demo.launch
   1) Move joints
   2) click plan and execute
   
   
3. RUN PODO with Qt editor
