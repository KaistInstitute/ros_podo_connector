/* =============Header file include ============= */
/*for ROS Action msg */
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"


/*Pre-defined Action msg for PODO motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>

/*for TCP/IP socket client */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/*for ROS additional msgs */
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>

/*for multi-threaded locks */
#include <mutex>
std::mutex variable_lock;


/* =============TCP/IP  global variables initialize ============= */
#define PODO_ADDR       "127.0.0.1"
#define PODO_PORT       5000

char ip[20];
int sock = 0;
struct sockaddr_in  server;

pthread_t LANTHREAD_t;
int threadWorking = true;
int connectionStatus = false;
int ON_publish = true;

//custom header RX/TX buffer
LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;

int     RXDataSize;
int     TXDataSize;
void*   RXBuffer;
void*   TXBuffer;
bool    IsdataRead = false;
/* ====================================================*/


#define ROW 50000
#define COL 10
int Save_Index = 0;
bool flag1 = false;
bool flag2 = false;
double Save_Data[COL][ROW];
FILE *fp;

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void*);
void    NewRXData();



float RSPdata[4000];
sensor_msgs::JointState joint_state;

	
//subscriber
ros::Subscriber joint_states_sub;
const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "WST", "RWH", "LWH", "BWH"
};


void joint_states_callback(const sensor_msgs::JointState& joint_state_msg){
   if(connectionStatus && ON_publish){
       std::cout<< "callback is called" << std::endl;

       TXData.ros2podo_data.CMD_JOINT = MODE_JOINT_PUBLISH;
       //Match Joint names
       for(int i=0; i< NUM_JOINTS; i++){ //joints in "joint_state" are listed in different order with the joints in joint_information.h
           for(int j=0; j< joint_state_msg.name.size(); j++){                           //go through all the joint names of "joint_states"
               if(JointBufferNameList[i] == joint_state_msg.name[j]){  // store the joint information only when joint name matches
                   double _moveitref = joint_state_msg.position[j];
                    TXData.ros2podo_data.Arm_action.joint[i].reference = _moveitref*R2Df;  //Joint Reference
                    TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = CONTROL_ON;                 //Move mode
                    write(sock, &TXData, TXDataSize);
               }
           }
       }
   }
}

void testJointMove()
{
    TXData.ros2podo_data.CMD_JOINT = MODE_MOVE_JOINT;

    TXData.ros2podo_data.Arm_action.joint[rosREB].reference = -30.0;
    TXData.ros2podo_data.Arm_action.joint[rosREB].ONOFF_control = CONTROL_ON;
    TXData.ros2podo_data.Arm_action.joint[rosREB].GoalmsTime = 3000;

    TXData.ros2podo_data.Arm_action.joint[rosLEB].reference = -30.0;
    TXData.ros2podo_data.Arm_action.joint[rosLEB].ONOFF_control = CONTROL_ON;
    TXData.ros2podo_data.Arm_action.joint[rosLEB].GoalmsTime = 3000;

    write(sock, &TXData, TXDataSize);
}

void testWBIK()
{
	    printf("write WBIK\n");

    TXData.ros2podo_data.CMD_JOINT = MODE_SET_WBIK;

    TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].ONOFF_movepos = CONTROL_ON;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[0] = 0.5;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[1] = -0.3;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[2] = 0.2;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].GoalmsTime = 2000;


    TXData.ros2podo_data.Arm_action.wbik[RIGHT_ELBOW].ONOFF_movepos = CONTROL_ON;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_ELBOW].Goal_angle = -30.0;
    TXData.ros2podo_data.Arm_action.wbik[RIGHT_ELBOW].GoalmsTime = 2000;


    TXData.ros2podo_data.Arm_action.wbik[WAIST].ONOFF_movepos = CONTROL_ON;
    TXData.ros2podo_data.Arm_action.wbik[WAIST].Goal_angle = -30.0;
    TXData.ros2podo_data.Arm_action.wbik[WAIST].GoalmsTime = 2000;

    write(sock, &TXData, TXDataSize);
    ROS_INFO("flag set false: %i\n",flag1);
}

void testWheel()
{
    TXData.ros2podo_data.CMD_WHEEL = WHEEL_MOVE_START;
    printf("testwheel\n");
    TXData.ros2podo_data.Base_action.wheel.MoveX = 0.0;
    TXData.ros2podo_data.Base_action.wheel.MoveY = 0.0;
    TXData.ros2podo_data.Base_action.wheel.ThetaDeg = 180.0*D2Rf;

    write(sock, &TXData, TXDataSize);
}

void testGripper()
{
    TXData.ros2podo_data.CMD_GRIPPER = GRIPPER_OPEN;
    TXData.ros2podo_data.Gripper_action.mode = GRIPPER_BOTH;

    write(sock, &TXData, TXDataSize);
}

void testJointPublishRSP()
{
    if(IsdataRead == true)
    {
        static int cntRSP = 0;
        if(cntRSP < 4000)
        {
            ROS_INFO("data = %f\n",RSPdata[cntRSP]);
            TXData.ros2podo_data.CMD_JOINT = MODE_JOINT_PUBLISH;
            TXData.ros2podo_data.Arm_action.joint[rosRSP].reference = RSPdata[cntRSP];
            TXData.ros2podo_data.Arm_action.joint[rosRSP].ONOFF_control = CONTROL_ON;
            write(sock, &TXData, TXDataSize);
            cntRSP++;
        }
    }

}

void e_stop()
{
    TXData.ros2podo_data.CMD_JOINT = MODE_E_STOP;
    ON_publish = false;
    write(sock, &TXData, TXDataSize);

}

void wheel_stop()
{
    TXData.ros2podo_data.CMD_WHEEL = WHEEL_MOVE_STOP;
    write(sock, &TXData, TXDataSize);

}

void print_encoder_ref()
{
    for(int i=0;i<NUM_JOINTS;i++)
    {
        printf("%f ",RXData.podo2ros_data.sensor.ENCODER[i].CurrentReference);
    }

    printf("\n");
}


void printf_feedback()
{
    //Arm feedback print
    printf("||============================================||\n");

//    printf("  Action Goal[%d] : %f, %f, %f\n", TXData.ros2podo_data.index,TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[0],TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[1],TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[2]);
//    printf("  Feedback   [%d] : %f, %f, %f\n", RXData.podo2ros_data.index,RXData.podo2ros_data.Arm_feedback.wbik[RIGHT_HAND].Goal_pos[0],RXData.podo2ros_data.Arm_feedback.wbik[RIGHT_HAND].Goal_pos[1],RXData.podo2ros_data.Arm_feedback.wbik[RIGHT_HAND].Goal_pos[2]);

    printf("  Action Goal[%d] : %f\n",TXData.ros2podo_data.index, TXData.ros2podo_data.Arm_action.joint[rosREB].reference);
    printf("  Feedback   [%d] : %f\n",RXData.podo2ros_data.index, RXData.podo2ros_data.Arm_feedback.joint[rosREB].reference);

}

void save_data()
{
    if(Save_Index < ROW)
    {
        Save_Data[0][Save_Index] = RXData.podo2ros_data.index;
        Save_Data[1][Save_Index] = TXData.ros2podo_data.index;

        Save_Data[2][Save_Index] = TXData.ros2podo_data.Arm_action.joint[REB].reference;
        Save_Data[3][Save_Index] = TXData.ros2podo_data.Arm_action.wbik[RIGHT_HAND].Goal_pos[0];
        Save_Data[4][Save_Index] = TXData.ros2podo_data.Base_action.wheel.MoveX;

        Save_Data[5][Save_Index] = RXData.podo2ros_data.Arm_feedback.joint[REB].reference;
        Save_Data[6][Save_Index] = RXData.podo2ros_data.Arm_feedback.wbik[RIGHT_HAND].Goal_pos[0];
        Save_Data[7][Save_Index] = RXData.podo2ros_data.Base_feedback.wheel.MoveX;

        Save_Index++;
        if(Save_Index >= ROW) Save_Index = 0;
    }
}

void make_txt()
{
    printf("make_txt\n");
    fp = fopen("/home/rainbow/Desktop/dataROS.txt","w");

    printf("file open, index = %d\n",Save_Index);
    for(int ib=0;ib<Save_Index;ib++)
    {
//        printf("%d",ib);
        for(int j=0;j<COL;j++)
        {
            fprintf(fp,"%g\t", Save_Data[j][ib]);

        }
        fprintf(fp,"\n");
    }
//    fprintf(fp, "hi");
    printf("file write\n");
    fclose(fp);

    TXData.ros2podo_data.CMD_JOINT = MODE_SAVE;
    write(sock, &TXData, TXDataSize);
}

/*write TX*/
void writeTX()
{
    printf("writing given TX\n");
    write(sock, &TXData, TXDataSize);
}

/*clear TXBuffer to default values*/
void clearTXBuffer()
{
	//std::lock_guard<std::mutex> lock(variable_lock);
	//reset command
	TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(0);
	TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(0);
	TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(0);
	
	
	//reset action data
	//TXData.ros2podo_data.Arm_action.joint[NUM_JOINTS] 
	//TXData.ros2podo_data.Arm_action.wbik[NUM_PARTS] 
	TXData.ros2podo_data.Arm_action.result_flag = 0;
	
	//TXData.ros2podo_data.Base_action.wheel.MoveX = 0;
	//TXData.ros2podo_data.Base_action.wheel.MoveY = 0;
	//TXData.ros2podo_data.Base_action.wheel.ThetaDeg = 0;
	TXData.ros2podo_data.Base_action.result_flag = 0;
	
	//TXData.ros2podo_data.Gripper_action.mode = 0;
	//TXData.ros2podo_data. = static_cast<GRIPPER_PARAMETER>(0);
	TXData.ros2podo_data.Gripper_action.result_flag = 0;
	
}

/*clear RXBuffer to default values*/
void clearRXBuffer()
{
	//std::lock_guard<std::mutex> lock(variable_lock);
	//reset done flag
	RXData.podo2ros_data.Arm_feedback.result_flag = 0;
	RXData.podo2ros_data.Base_feedback.result_flag = 0;

	

}



/*========================== Start of Base Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_BaseAction
{
protected:

  ros::NodeHandle nh_base;
  actionlib::SimpleActionServer<ros_podo_connector::RosPODO_BaseAction> asBase_; 
  std::string action_name_;

  bool baseMotionSuccess = false;
  bool motionStarted = false;
  	
  // create messages that are used to published feedback&result
  ros_podo_connector::RosPODO_BaseFeedback feedback_;
  ros_podo_connector::RosPODO_BaseResult result_;
  
  //timer start
  ros::Time beginTime = ros::Time::now();

public:

  RosPODO_BaseAction(std::string name) :
    asBase_(nh_base, name, boost::bind(&RosPODO_BaseAction::executeCB, this, _1), false),
    action_name_(name)
  {
    asBase_.start();
  }

  ~RosPODO_BaseAction(void)
  {
	  
  }

/*Call Back function when goal is received from Action client*/
  void executeCB(const ros_podo_connector::RosPODO_BaseGoalConstPtr &goal)
  {
	clearTXBuffer();

    // loop this thread to check status of goal
    ros::Rate r(200);
    

    
    // publish info to the console for the user
    ROS_INFO("%s: 1 Received Base Motion with Command: %i\n", action_name_.c_str(), goal->wheelmove_cmd);
    
    //=====execute action for robot motion========
    
    //write TX to podo
    	
    TXData.ros2podo_data.CMD_WHEEL = static_cast<WHEELMOVE_CMD>(goal->wheelmove_cmd);
    TXData.ros2podo_data.Base_action.wheel.MoveX = goal->MoveX;
    TXData.ros2podo_data.Base_action.wheel.MoveY = goal->MoveY;
    TXData.ros2podo_data.Base_action.wheel.ThetaDeg = goal->ThetaDeg;
    write(sock, &TXData, TXDataSize);
    
    ros::Duration(5).sleep();
    asBase_.setSucceeded(result_);
    
    /* test goal loop
    //while loop to check until goal is finished
    while(baseMotionSuccess == false)
    {
		r.sleep();
	}
	ROS_INFO("base action done: %i\n", baseMotionSuccess); 
	asBase_.setSucceeded(result_);
	*/

  }

  
  /* update feedback action topic*/
  void publishFeedback()
  {
	 feedback_.MoveX = RXData.podo2ros_data.Base_feedback.wheel.MoveX;
	 feedback_.MoveY = RXData.podo2ros_data.Base_feedback.wheel.MoveY;
	 feedback_.ThetaDeg = RXData.podo2ros_data.Base_feedback.wheel.ThetaDeg;
	 asBase_.publishFeedback(feedback_);
	 
	 //ROS_INFO("MoveX RX: %f\n", RXData.podo2ros_data.Base_feedback.wheel.MoveX); 
  }
  
  /* update result action topic*/
  void publishResult()
  {
	  
	  if(motionStarted == true && RXData.podo2ros_data.Base_feedback.result_flag)
	  {
		  
		  result_.MoveX = feedback_.MoveX;
		  result_.MoveY = feedback_.MoveY;
		  result_.ThetaDeg = feedback_.ThetaDeg;
		  result_.result_flag = 1;
		  ROS_INFO("Finished base action: %i\n", result_.result_flag ); 
		  baseMotionSuccess = true;
  
	  }
	  
  }
  
    //1 if alive, 0 else
  int returnServerStatus()
  {
	  if(asBase_.isActive()) { return 1; }
	  else { return 0; }

  }

};
/*========================== End of Base Action Server ==================================*/


/*========================== Start of Arm Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_ArmAction
{
protected:

  ros::NodeHandle nh_arm;
  actionlib::SimpleActionServer<ros_podo_connector::RosPODO_ArmAction> asArm_; 
  std::string action_name_;
  bool armMotionSuccess = false;
  bool motionStarted = false;
  
  // create messages that are used to published feedback&result
  ros_podo_connector::RosPODO_ArmFeedback feedback_;
  ros_podo_connector::RosPODO_ArmResult result_;
  
  //timer start
  ros::Time beginTime = ros::Time::now();

public:

  RosPODO_ArmAction(std::string name) :
    asArm_(nh_arm, name, boost::bind(&RosPODO_ArmAction::executeCB, this, _1), false),
    action_name_(name)
  {
    asArm_.start();
  }

  ~RosPODO_ArmAction(void)
  {
	  
  }

/*Call Back function when goal is received from Action client*/
  void executeCB(const ros_podo_connector::RosPODO_ArmGoalConstPtr &goal)
  {
	clearTXBuffer();

    // helper variables
    ros::Rate r(200);
    

    
    //=====execute action for robot motion========
    TXData.ros2podo_data.CMD_JOINT = static_cast<JOINTMOVE_CMD>(goal->jointmove_cmd);
    
    
    //Map the arm move .action to TXData for cases {publish joint, joint, wbik, estop, save}
    switch(static_cast<JOINTMOVE_CMD>(goal->jointmove_cmd))
    {
		case MODE_JOINT_PUBLISH:
		{
			ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
			break;
		}
		
		case MODE_MOVE_JOINT:
		{
			ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				TXData.ros2podo_data.Arm_action.joint[i].ONOFF_control = goal->joint_ref[i].OnOffControl;
				TXData.ros2podo_data.Arm_action.joint[i].reference = goal->joint_ref[i].reference;
				TXData.ros2podo_data.Arm_action.joint[i].GoalmsTime = goal->joint_ref[i].GoalmsTime;
			}
			break;
		}
		
		case MODE_SET_WBIK:
		{
			ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
			for(int i = 0; i < NUM_PARTS;  i++)
			{
				TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_movepos = goal->wbik_ref[i].OnOff_position;
				TXData.ros2podo_data.Arm_action.wbik[i].ONOFF_moveori = goal->wbik_ref[i].OnOff_orientation;
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[0] = goal->wbik_ref[i].goal_position[0];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[1] = goal->wbik_ref[i].goal_position[1];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_pos[2] = goal->wbik_ref[i].goal_position[2];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[0] = goal->wbik_ref[i].goal_orientation[0];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[1] = goal->wbik_ref[i].goal_orientation[1];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[2] = goal->wbik_ref[i].goal_orientation[2];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_quat[3] = goal->wbik_ref[i].goal_orientation[3];
				TXData.ros2podo_data.Arm_action.wbik[i].Goal_angle = goal->wbik_ref[i].goal_angle;
				TXData.ros2podo_data.Arm_action.wbik[i].GoalmsTime = goal->wbik_ref[i].GoalmsTime;
			}
			break;
		}
		
		case MODE_E_STOP:
		{
			ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
			break;
		}
		
		case MODE_SAVE:
		{
			ROS_INFO("%s: 1 Received Arm Motion with Command: %i\n", action_name_.c_str(), goal->jointmove_cmd);
			break;
		}
	}
    

	//write TXdata
    write(sock, &TXData, TXDataSize);
    ros::Duration(5).sleep();
    asArm_.setSucceeded(result_);
           
    
    /* test goal send
    //while loop to check until goal is finished
    while(armMotionSuccess == false)
    {
		r.sleep();
	}
	ROS_INFO("arm action done: %i\n", armMotionSuccess); 
	asArm_.setSucceeded(result_);
	*/ 

  }
  

     
  
  /* update feedback action topic*/
  void publishFeedback()
  {
	  //arm joint feedback
	  for(int i = 0; i < NUM_JOINTS; i++)
	  {
		  feedback_.joint_ref[i].OnOffControl = RXData.podo2ros_data.Arm_feedback.joint[i].ONOFF_control;
		  feedback_.joint_ref[i].reference = RXData.podo2ros_data.Arm_feedback.joint[i].reference;
		  feedback_.joint_ref[i].GoalmsTime = RXData.podo2ros_data.Arm_feedback.joint[i].GoalmsTime;

	  }
	  
	  //arm wbik feedback
	  for(int i = 0; i < NUM_PARTS;  i++)
	  {
		  feedback_.wbik_ref[i].OnOff_position = RXData.podo2ros_data.Arm_feedback.wbik[i].ONOFF_movepos;
		  feedback_.wbik_ref[i].OnOff_orientation = RXData.podo2ros_data.Arm_feedback.wbik[i].ONOFF_moveori;
		  feedback_.wbik_ref[i].goal_position[0] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[0];
		  feedback_.wbik_ref[i].goal_position[1] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[1];
		  feedback_.wbik_ref[i].goal_position[2] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_pos[2];
		  feedback_.wbik_ref[i].goal_orientation[0] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[0];
		  feedback_.wbik_ref[i].goal_orientation[1] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[1];
		  feedback_.wbik_ref[i].goal_orientation[2] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[2];
		  feedback_.wbik_ref[i].goal_orientation[3] = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_quat[3];
		  feedback_.wbik_ref[i].goal_angle = RXData.podo2ros_data.Arm_feedback.wbik[i].Goal_angle;
		  feedback_.wbik_ref[i].GoalmsTime = RXData.podo2ros_data.Arm_feedback.wbik[i].GoalmsTime;  
	  }
	  
	  //ADD JOINT PUBLISH
	  
	  
	//publish feedback
	  asArm_.publishFeedback(feedback_);
	 
  }
  
  /* update result action topic*/
  void publishResult()
  {  
	  //received done flag from PODO
	  if(motionStarted == true && RXData.podo2ros_data.Arm_feedback.result_flag)
	  {
		  
		  //arm joint feedback
		  for(int i = 0; i < NUM_JOINTS; i++)
		  {
			  result_.joint_ref[i].OnOffControl = feedback_.joint_ref[i].OnOffControl;
			  result_.joint_ref[i].reference = feedback_.joint_ref[i].reference;
			  result_.joint_ref[i].GoalmsTime = feedback_.joint_ref[i].GoalmsTime;

		  }
		  
		  
		  //arm wbik feedback
		  for(int i = 0; i < NUM_PARTS;  i++)
		  {
			  result_.wbik_ref[i].OnOff_position = feedback_.wbik_ref[i].OnOff_position;
			  result_.wbik_ref[i].OnOff_orientation = feedback_.wbik_ref[i].OnOff_orientation;
			  result_.wbik_ref[i].goal_position[0] = feedback_.wbik_ref[i].goal_position[0];
			  result_.wbik_ref[i].goal_position[1] = feedback_.wbik_ref[i].goal_position[1];
			  result_.wbik_ref[i].goal_position[2] = feedback_.wbik_ref[i].goal_position[2];
			  result_.wbik_ref[i].goal_orientation[0] = feedback_.wbik_ref[i].goal_orientation[0];
			  result_.wbik_ref[i].goal_orientation[1] = feedback_.wbik_ref[i].goal_orientation[1];
			  result_.wbik_ref[i].goal_orientation[2] = feedback_.wbik_ref[i].goal_orientation[2];
			  result_.wbik_ref[i].goal_orientation[3] = feedback_.wbik_ref[i].goal_orientation[3];
			  result_.wbik_ref[i].goal_angle = feedback_.wbik_ref[i].goal_angle;
			  result_.wbik_ref[i].GoalmsTime = feedback_.wbik_ref[i].GoalmsTime;  
		  }
	  
		  result_.result_flag = 1;
		  ROS_INFO("Finished arm action: %i\n", result_.result_flag ); 
		  armMotionSuccess = true;
  
	  }
	  
  }
  
    //1 if alive, 0 else
  int returnServerStatus()
  {
	  
	  if(asArm_.isActive()) { return 1; }
	  else { return 0; }

  }

};
/*========================== End of Arm Action Server ==================================*/

/*========================== Start of Gripper Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODO_GripperAction
{
protected:

  ros::NodeHandle nh_gripper;
  actionlib::SimpleActionServer<ros_podo_connector::RosPODO_GripperAction> asGripper_; 
  std::string action_name_;
  bool motionStarted = false;
  
  // create messages that are used to published feedback&result
  ros_podo_connector::RosPODO_GripperFeedback feedback_;
  ros_podo_connector::RosPODO_GripperResult result_;
  
  //timer start
  ros::Time beginTime = ros::Time::now();

public:

  RosPODO_GripperAction(std::string name) :
    asGripper_(nh_gripper, name, boost::bind(&RosPODO_GripperAction::executeCB, this, _1), false),
    action_name_(name)
  {
    asGripper_.start();
  }

  ~RosPODO_GripperAction(void)
  {
  }

  /*Call Back function when goal is received from Action client*/
  void executeCB(const ros_podo_connector::RosPODO_GripperGoalConstPtr &goal)
  {
	clearTXBuffer();

    // helper variables
    ros::Rate r(1);
    bool success = true;

    
    // publish info to the console for the user
    ROS_INFO("%s: 1 Received Gripper Motion with Command: %i\n", action_name_.c_str(), goal->grippermove_cmd);
    
    //=====execute action for robot motion========
    
    //write TX to podo
    
    TXData.ros2podo_data.CMD_GRIPPER = static_cast<GRIPPERMOVE_CMD>(goal->grippermove_cmd);
    TXData.ros2podo_data.Gripper_action.mode = goal->mode;
    
    write(sock, &TXData, TXDataSize);

      asGripper_.setSucceeded(result_);
      flag2 = true;
      
    
  }
  
  void publishFeedback()
  {
	 
  }
  
  void publishResult()
  {
	  
  }
  
  

};
/*========================== End of Gripper Action Server ==================================*/



/*Print helpful summary about this code */
void printInitialInfo()
{
	std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Starting ROS2PODO Motion Action Server" << std::endl << std::endl;
    std::cout << "   Developer: Moonyoung Lee" << std::endl;
    std::cout << "   E-mail   : ml634@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;
}

/*========================== LAN Communication Functions =============================*/
/*Load IP txt file for TCP/IP TX/RX */
/*Create Socket and initialize TX/RX size */
int initializeSocket()
{
 FILE *fpNet = NULL;
    FILE *data = NULL;
    data = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/src/ROScommandData.txt","r");
    fpNet = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }
    if(data == NULL)
    {
        std::cout << "No file founded" << std::endl;

    }else
    {
        std::cout << "file found" << std::endl;
        IsdataRead = true;
        for(int i=0;i<4000;i++)
        {
            fscanf(data, "%f ", &RSPdata[i]);
        }
        fclose(data);
    }


    if(CreateSocket(ip, PODO_PORT)){
        ROS_INFO("Created Socket..");

        RXDataSize = sizeof(LAN_PODO2ROS);
        TXDataSize = sizeof(LAN_ROS2PODO);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        
        /*
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0){
            ROS_ERROR("Create Thread Error..");
            return 0;
        }*/
    }
    else{
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node..");
        return 0;
    }    
    
}


int CreateSocket(const char *addr, int port){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }
    server.sin_addr.s_addr = inet_addr(addr);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int Connect2Server()
{
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        std::cout << " Connection Failed" << std::endl;
        return false;
    }
    std::cout << "Client connect to server!! (PODO_CONNECTOR)" << std::endl;
    return true;
}

void testLAN()
{
	
	static unsigned int tcp_status = 0x00;
    static int tcp_size = 0;
    static int connectCnt = 0;

    if(threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(sock == 0){
                CreateSocket(ip, PODO_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    //std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            tcp_size = read(sock, RXBuffer, RXDataSize);
            if(tcp_size == RXDataSize){
                memcpy(&RXData, RXBuffer, RXDataSize);
            }

            if(tcp_size == 0){
                tcp_status = 0x00;
                connectionStatus = false;
                close(sock);
                sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
    }
}

void* LANThread(void *){
    threadWorking = true;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    int connectCnt = 0;

    while(threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(sock == 0){
                CreateSocket(ip, PODO_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    //std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            tcp_size = read(sock, RXBuffer, RXDataSize);
            if(tcp_size == RXDataSize){
                memcpy(&RXData, RXBuffer, RXDataSize);
            }

            if(tcp_size == 0){
                tcp_status = 0x00;
                connectionStatus = false;
                close(sock);
                sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
    }
    return NULL;
}
/*========================== End of LAN functions ==================================*/


/*========================== main while loop ==================================*/
int main(int argc, char **argv)
{
	printInitialInfo();
	//Initialize ROS node
	ros::init(argc, argv, "ros_podo_connector");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    
    // Create Socket 
	initializeSocket();
	
   //Initialize ROS Action Server
	RosPODO_BaseAction rospodo_base("rospodo_base");
	RosPODO_ArmAction rospodo_arm("rospodo_arm");
	RosPODO_GripperAction rospodo_gripper("rospodo_gripper");
	ROS_INFO("Starting ROS2PODO Action Servers");
	
    
   
	/* === main while loop to RX feedback calls at regular periods === */
    while(ros::ok())
    {
		
		testLAN();

		//check if action server is active
        if(rospodo_base.returnServerStatus())
        {
			rospodo_base.publishFeedback();
			rospodo_base.publishResult();
		}
		
		if(rospodo_arm.returnServerStatus())
        {
			rospodo_arm.publishFeedback();
			rospodo_arm.publishResult();
		}
		

		
        if( flag2 == true)
        {
            //ROS_INFO("flag is: %i\n",flag2);
            //flag1 = true;
//            testJointMove();
            //testWheel();
            //writeTX();
//            e_stop();
//            wheel_stop();
//            testGripper();
            //testWBIK();
           //printf_feedback();
            //make_txt();
            flag2 = false;

        }
        
        //save_data();
        //print_encoder_ref();
        
        TXData.ros2podo_data.index += 1;
        //testJointPublishRSP();

		//loop at desired rate 
		ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
/*========================== End of Code ==================================*/
