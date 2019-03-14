# ros_podo_connector(ROS2PODO)

A ros node "ros_podo_connector" is developed for the users who are not familiar with PODO frameworks.

Since the high level program(intelligence) developers are usually more familiar with ROS environment, it is easier to move robot through a ros node. However, it is impossible to run Mobile hubo with out HUBO LAB's private robot framework, PODO, a node which connects ros environment and PODO frameworks.

"ros_podo_connector" subscribes topic "joint_state", a desired joint angles, and deliver it to PODO framework through TCP/IP communication.

To verfy this ros node you need
* ros project "ros_podo_connector"
* PODO
* mobile hubo moveit configuration (This can be replaced any ros node which publish joint_state) 


1. RUN "ros_podo_connector"
 $ ros core
 $ rosrun ros_podo_connector ros_podo_connector
