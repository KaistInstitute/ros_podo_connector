#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
#include <ros_podo_connector/RosPODO_TrajAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"

enum TRAJECTORY_CMD
{
    MOVE_ABSOLUTE = 0,
    MOVE_RELATIVE
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "traj_request_client");

    // create the action client
    // true causes the client to spin its own thread
//    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
//    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
//    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_TrajAction> ac_traj("rospodo_traj", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    //  ac_base.waitForServer(); //will wait for infinite time
    //  ac_arm.waitForServer(); //will wait for infinite time
    //  ac_gripper.waitForServer(); //will wait for infinite time
    ac_traj.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // send a goal to the action
//    ros_podo_connector::RosPODO_BaseGoal      goal_base;
//    ros_podo_connector::RosPODO_BaseGoal      goal_base_return;
//    ros_podo_connector::RosPODO_ArmGoal       goal_arm;
//    ros_podo_connector::RosPODO_GripperGoal   goal_gripper;

    ros_podo_connector::RosPODO_TrajGoal      goal_traj;


    /* ============== Move Endeffector to ABSOLUTE COORDINATES  ==============  */
//    goal_traj.traj_cmd = MOVE_ABSOLUTE;
//    goal_traj.x = 0.5;
//    goal_traj.y = 0.5;
//    goal_traj.z = 0.8;
//    goal_traj.ThetaDeg = 1.0;

//    ac_traj.sendGoal(goal_traj);
//    ros::Duration(6).sleep();

//    //Debug
//    std::cout << "Sent Goal: " << std::endl;
//    std::cout << "x: " << goal_traj.x << std::endl;
//    std::cout << "y: " << goal_traj.y << std::endl;
//    std::cout << "z: " << goal_traj.z << std::endl;
//    std::cout << "theta: " << goal_traj.ThetaDeg << std::endl;
//    std::cout << "mode: " << goal_traj.traj_cmd << std::endl;

    /* ============== Move Endeffector  ==============  */
    goal_traj.traj_cmd = MOVE_RELATIVE;

    goal_traj.x = 0.1; //0.1;
    goal_traj.y = 0.2;
    goal_traj.z = 0.2; //0.0;
    goal_traj.ori_w = 0.0;
    goal_traj.ori_x = 0.0;
    goal_traj.ori_y = 0.0;
    goal_traj.ori_z = 0.0;
    goal_traj.planGroup = "L_arm";

    ac_traj.sendGoal(goal_traj);
    ros::Duration(6).sleep();

    //Debug
    std::cout << "Sent Goal: " << std::endl;
    std::cout << "x: " << goal_traj.x << std::endl;
    std::cout << "y: " << goal_traj.y << std::endl;
    std::cout << "z: " << goal_traj.z << std::endl;
    std::cout << "ori_w: " << goal_traj.ori_w << std::endl;
    std::cout << "ori_x: " << goal_traj.ori_x << std::endl;
    std::cout << "ori_y: " << goal_traj.ori_y << std::endl;
    std::cout << "ori_z: " << goal_traj.ori_z << std::endl;
    std::cout << "mode: " << goal_traj.traj_cmd << std::endl;



    //wait for the action to return
    bool finished_before_timeout = ac_traj.waitForResult(ros::Duration(100.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_traj.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}
