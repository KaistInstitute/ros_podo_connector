#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"


#include <sensor_msgs/JointState.h>

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

//variables
int loop_cnt=0; //loop counter
int prev_cnt=0; //save loop counter for calculation.
double prev_ref[NUM_JOINTS]={0,};
double curr_ref[NUM_JOINTS]={0,};

int CB_flag = 0;

//subscriber
ros::Subscriber joint_states_sub;
const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};

void TX_traj_interpolate();


void joint_states_callback(const sensor_msgs::JointState& joint_state_msg){

    //debugging
   // std::cout<< "callback is called" << std::endl;

    CB_flag++;
    prev_cnt = loop_cnt;
    loop_cnt = 0;

    for(int i=0; i<NUM_JOINTS; i++)
        prev_ref[i] = curr_ref[i];

    //Match Joint names
    for(int i=0; i< NUM_JOINTS; i++){ //joints in "joint_state" are listed in different order with the joints in joint_information.h
        for(int j=0; j< joint_state_msg.name.size(); j++){                           //go through all the joint names of "joint_states"
            if(JointBufferNameList[i] == joint_state_msg.name[j]){                   // store the joint information only when joint name matches
                curr_ref[i] = joint_state_msg.position[j];
        }
    }
}


}




int main (int argc, char **argv)
{
    ros::init(argc, argv, "joint_publish_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_base.waitForServer();      //will wait for infinite time
  ac_arm.waitForServer();       //will wait for infinite time
  ac_gripper.waitForServer();   //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  // create goal instance
  ros_podo_connector::RosPODO_BaseGoal      goal_base;
  ros_podo_connector::RosPODO_ArmGoal       goal_arm;
  ros_podo_connector::RosPODO_GripperGoal   goal_gripper;



  /* ============== Arm Data Action JOINT PUBLISH ==============  */
  ros::Subscriber joint_states_sub;
  ros::NodeHandle n;
  joint_states_sub = n.subscribe("joint_states", 100, joint_states_callback);

  ros::Time tzero(0);
  ros::Time beginTime = ros::Time::now();
  static int cnt = 0;
  ros::Rate loop_rate(200);

  double d_ref[NUM_JOINTS-3];

  //Subscribe topic "joint_states"
  while(ros::ok())
  {
      ros::spinOnce();

      if(CB_flag == 1){
          for(int i=0; i<NUM_JOINTS-3; i++){
              goal_arm.joint_ref[i].reference = curr_ref[i]*R2Df;
              goal_arm.joint_ref[i].OnOffControl = CONTROL_ON;
          }
          goal_arm.jointmove_cmd = MODE_JOINT_PUBLISH;

          //Send Goal
          ac_arm.sendGoal(goal_arm);
      }

      if(CB_flag >1) // if CB is called more than once
      {
          goal_arm.jointmove_cmd = MODE_JOINT_PUBLISH;

          //interpolate
          for(int i=0; i<NUM_JOINTS-3; i++){
              d_ref[i] = (curr_ref[i] - prev_ref[i]) / prev_cnt;
              goal_arm.joint_ref[i].reference += d_ref[i]*R2Df;
              goal_arm.joint_ref[i].OnOffControl = CONTROL_ON;

          }

          //Send Goal
          ac_arm.sendGoal(goal_arm);

      }


      loop_cnt++;
      //std::cout << "Loop Counter = " << loop_cnt << std::endl;

      loop_rate.sleep();
  }

  //exit
  return 0;
}
