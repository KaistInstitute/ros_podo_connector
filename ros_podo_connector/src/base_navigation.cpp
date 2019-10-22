#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>

/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"
/* ROS package include */
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>

#include <sensor_msgs/JointState.h>

/* transformation headers*/
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "tf/transform_datatypes.h"

#include <unistd.h>

/* include navi action file */
#include <mobile_path_planning/naviAction.h>

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

#define arrival_threshold 2 //arrival grid size
#define averageWindowN 5    //sample number for low-pass filter
#define grid_resolution 0.1 //meter
#define marker_offset_z 0.5

/* ========= global variables ========== */
ros::Publisher path_pub; //path difference

float velocityxyzw [4][averageWindowN];
float vx_tx, vy_tx, vw_tx, vz_tx;

float sum_of_i = 0;

int received_path = 0;
int arrived_goal = 0;
int use_aruco_marker = 1;
int move_flag = 0;
int have_aligned = 0;
int marker_detected = 0;

int orientation_align_start = 0;
int orientation_aligned_finished = 0;

//marker pose related variables
float marker_x, marker_y, marker_z;
float marker_robot_x, marker_robot_y, marker_robot_z;
float roll_c, pitch_c, yaw_c;
float yaw_goal_deg;
float yaw_goal_rad;

ros::Time currentTime, beginTime;



geometry_msgs::Pose temporary_goal_pose;

/* ======================================= */


/* receive goal pose from Navi Action */
void receive_goal_pose_action(const mobile_path_planning::naviActionGoalConstPtr &goal)
{

	orientation_align_start = 1;
    orientation_aligned_finished = 0;
    
    temporary_goal_pose.position.x = goal->goal.pose_x;
    temporary_goal_pose.position.y = goal->goal.pose_y;
    temporary_goal_pose.position.z = goal->goal.pose_z;
    temporary_goal_pose.orientation.x = goal->goal.ori_x;
    temporary_goal_pose.orientation.y = goal->goal.ori_y;
    temporary_goal_pose.orientation.z = goal->goal.ori_z;
    temporary_goal_pose.orientation.w = goal->goal.ori_w;
    
    
	//convert quaternion to euler
	tf::Quaternion quat_goal;
	tf::quaternionMsgToTF(temporary_goal_pose.orientation, quat_goal);
	double roll_goal, pitch_goal, yaw_goal;
	tf::Matrix3x3(quat_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
	
	yaw_goal_rad = yaw_goal;
	yaw_goal_deg = yaw_goal * R2Df;
	
	ROS_INFO("NEW GOAL from Action!! goalX: %f, goalY: %f, goalTheta: %f",temporary_goal_pose.position.x, temporary_goal_pose.position.y, yaw_goal_deg);

}

/* receive goal pose from RVIZ (duplicate from path plan) */
void receive_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    orientation_align_start = 1;
    orientation_aligned_finished = 0;
    temporary_goal_pose = msg->pose;
    
    
    
    //convert quaternion to euler
	tf::Quaternion quat_goal;
	tf::quaternionMsgToTF(msg->pose.orientation, quat_goal);
	double roll_goal, pitch_goal, yaw_goal;
	tf::Matrix3x3(quat_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
	
	yaw_goal_rad = yaw_goal;
	yaw_goal_deg = yaw_goal * R2Df;
	
	ROS_INFO("NEW GOAL from RViz!! goalX: %f, goalY: %f, goalTheta: %f",msg->pose.position.x, msg->pose.position.y, yaw_goal_deg);

}

/* map base velocity command from path size in 2D */
void velocity_from_path(const nav_msgs::Path::ConstPtr& msg)
{
    //ROS_INFO(" ============= received path CB =============");
    
     /*check for non-empty vector*/
    if(msg->poses.empty())
    {
		ROS_ERROR(" empty path");
		return;
	}
    

    int pathSize = msg->poses.size();
    //ROS_INFO("path size: %d", pathSize); 
    
    received_path = 1;
    
    if(pathSize > arrival_threshold)
    {
		arrived_goal = 0;
		move_flag = 1;
		have_aligned = 0;
	}
	
        nav_msgs::Path pathDifferece; //vectory of differences in path
    geometry_msgs::PoseStamped temporaryPose; //used to fill
    
    //create pose difference array 
    for(int i = 0; i < pathSize-1; i++)
    {
		temporaryPose.pose.position.x = msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x;
		temporaryPose.pose.position.y = msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y;
		temporaryPose.pose.orientation.z = msg->poses[i+1].pose.orientation.z -msg->poses[i].pose.orientation.z;
		temporaryPose.pose.orientation.w = msg->poses[i+1].pose.orientation.w -msg->poses[i].pose.orientation.w;
		
		pathDifferece.poses.push_back(temporaryPose);
	}
    
    int pathSize2 = pathDifferece.poses.size();
    //ROS_INFO("path2 size: %d", pathDifferece.poses.size()); 
    
    path_pub.publish(pathDifferece);
    
    
    /* deltaX range [0.0~0.1]m */
    /* velocity range [0.0~1.0] */
    /* linearly scale delta to velocity output range*/
    float vx_ref, vy_ref, vz_ref, vw_ref;
    float vx_out, vy_out, vz_out, vw_out;
    
    
    //many samples remainining in path
    if (pathSize > averageWindowN ) 
    {
		
			if(pathDifferece.poses.empty())
		{
			ROS_ERROR("Difference path  empty!!!");
			return;
		}
		
		//ROS_INFO("Difference path size: %d", pathSize2); 
		
	
		for (int i =0; i < averageWindowN; i++) //sum the next window of path differences
		{
			vx_ref = vx_ref + pathDifferece.poses[i].pose.position.x;
			vy_ref = vy_ref + pathDifferece.poses[i].pose.position.y;
			vw_ref = vw_ref + pathDifferece.poses[i].pose.orientation.z;
			vz_ref = vz_ref + pathDifferece.poses[i].pose.orientation.w;
		}
		
			//average and then scale to unitless velocity
		vx_ref = vx_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vy_ref = vy_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vz_ref = vz_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		vw_ref = vw_ref / averageWindowN * (1.0-0.0)/grid_resolution;
		
		//ROS_INFO("vx_ref: %f, vy_ref: %f\n", vx_ref, vy_ref); 
		
		//new reference at [end]
		velocityxyzw [0][averageWindowN-1] = vx_ref;
		velocityxyzw [1][averageWindowN-1] = vy_ref;
		velocityxyzw [2][averageWindowN-1] = vz_ref;
		velocityxyzw [3][averageWindowN-1] = vw_ref;
		
		
		/* to weight the LPF values by recent ones*/
		sum_of_i = 0;
		
		for(int i =1; i < averageWindowN+1; i++)
		{
			sum_of_i = sum_of_i + i;
			 
		}
			//ramp velocity using low-pass filter window
		for(int i =0; i < averageWindowN; i++)
		{
			
			vx_out = vx_out + ((i+1)/float(sum_of_i))*velocityxyzw[0][i];
			vy_out = vy_out + ((i+1)/float(sum_of_i))*velocityxyzw[1][i];
			vz_out = vz_out + ((i+1)/float(sum_of_i))*velocityxyzw[2][i];
			vw_out = vw_out + ((i+1)/float(sum_of_i))*velocityxyzw[3][i];
			
		}
	
		//update window
		for (int i =0; i < averageWindowN-1; i++)
		{
			velocityxyzw [0][i] = velocityxyzw [0][i+1];
			velocityxyzw [1][i] = velocityxyzw [1][i+1];
			velocityxyzw [2][i] = velocityxyzw [2][i+1];
			velocityxyzw [3][i] = velocityxyzw [3][i+1];
		}
	
	}
	
	
	//less than averageWindowN samples remaining in path. slow down
	else 
	{
		
		//slowing down
		if (pathSize > arrival_threshold)
		{
			 
			vx_out = pathDifferece.poses[0].pose.position.x * pathSize; // [-0.1 ~ 0.1] * [1 ~ 5]
			vy_out = pathDifferece.poses[0].pose.position.y * pathSize;
			vz_out = pathDifferece.poses[0].pose.orientation.z * pathSize;
			vw_out = pathDifferece.poses[0].pose.orientation.w * pathSize;
		}
		
		//arrived at goal
		else
		{
			vx_out = 0;
			vy_out = 0;
			vz_out = 0;
			vw_out = 0;
			//ROS_INFO("FINISHED STOP");
			arrived_goal = 1;
	
		}
	}
	

	/* bound max values*/
	if (vx_out <= 1.0 and vx_out >= -1.0) vx_tx = vx_out;
	else if(vx_out > 1.0 ) 		vx_tx = 1.0;
	else if(vx_out < -1.0) 	vx_tx = -1.0;
	else vx_tx = 0;
	
	if(vy_out <= 1.0 and vy_out >= -1.0) 	vy_tx = vy_out;
	else if(vy_out > 1.0) 		vy_tx = 1.0;
	else if(vy_out < -1.0) 	vy_tx = -1.0;
	else vy_tx = 0;
	
	
	
	
	/* handle nan or inf */
	//ROS_INFO("vx: %f, vy: %f\n", vx_tx, vy_tx); 
	
}



/* receive user request to use aruco marker for alignment */
void use_marker(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("will be using marker alignment");
	use_aruco_marker = 1;

}



/* align robot with aruco marker */
void position_from_marker(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	
	//move only when arrived at goal and user requested to use marker alignment
	if( (arrived_goal == 1) && (use_aruco_marker == 1) )
	{
		//ROS_INFO("detected marker!");
		marker_detected = 1;
		//ROS_INFO("x: %.3f, y: %.3f, z:%.3f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
		marker_x = msg->pose.position.x;
		marker_y = msg->pose.position.y;
		marker_z = msg->pose.position.z;
		
		//convert marker to robot position in 2D application
		marker_robot_y = -1 * marker_x;
		marker_robot_x = -marker_offset_z + marker_z;
		
		
		//convert quaternion to euler
		tf::Quaternion quat;
		tf::quaternionMsgToTF(msg->pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		
		roll_c = roll * R2Df;
		pitch_c = -1*pitch;
		yaw_c = yaw * R2Df;
    
    
		//ROS_INFO("r: %.3f, p: %.3f, y: %.3f\n", roll_c, pitch_c, yaw_c); 

		//ROS_INFO("x: %f, y: %f, z:%f, xt: %f, yt: %f, zt:%f", msg.pose.position.x, );
			//TF transformation from camera to robot pose
			
			/*
			 * xr = xc (
			 * yr = yc
			 * zr = zc
			 * 
			 * 
			 * /
	*/		
			
	}

}





int main (int argc, char **argv)
{
    ros::init(argc, argv, "moving_base_test");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
   
    ROS_INFO("Waiting for action server to start LMY");
    // wait for the action server to start
    //ac_base.waitForServer();      //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // create goal instance
    ros_podo_connector::RosPODO_BaseGoal      goal_base;


    /* ============== Initialize ==============  */
    ros::NodeHandle n;
    ros::Subscriber key_input_sub = n.subscribe("/mobile_hubo/navigation_path",100,velocity_from_path);
    ros::Subscriber marker_sub = n.subscribe("/aruco_single/pose",10,position_from_marker);
    
    //goal subscriber for 1. RVIZ 2. Navi Action
    ros::Subscriber goal_pose_subscriber = n.subscribe("/move_base_simple/goal", 10, receive_goal_pose);
    ros::Subscriber goal_pose_action_subscriber = n.subscribe("/navi_dummy/goal", 10, receive_goal_pose_action);
    
    //publish after align with marker
    ros::Publisher arrived_publisher = n.advertise<geometry_msgs::PoseStamped>("mobile_hubo/arrived_path",10);
    
    
    ros::Rate loop_rate(5);


    while(ros::ok())
    {
		
        ros::spinOnce();
        goal_base.VelX = 0;
        goal_base.VelY = 0;
            
        /* start sending velocity command */
        if( received_path == 1 && move_flag == 1)
        {
			goal_base.wheelmove_cmd = WHEEL_MOVE_VELOCITY;
			goal_base.VelX = vx_tx;
            goal_base.VelY = vy_tx;
			ROS_INFO("vx: %f, vy: %f\n", goal_base.VelX, goal_base.VelY); 
            
            ac_base.sendGoal(goal_base);
            
            if(arrived_goal == 1)
            {
				move_flag = 0;
				goal_base.wheelmove_cmd = WHEEL_MOVE_STOP;
				goal_base.VelX = 0;
				goal_base.VelY = 0;
				ROS_INFO("STOPPING velocity movement");
				ac_base.sendGoal(goal_base);
			
				
			}
			       
		}
		
		received_path = 0;
        
        /*
        if(orientation_align_start == 1)
        {
			goal_base.wheelmove_cmd = WHEEL_MOVE_START;
			goal_base.VelX = 0;
			goal_base.VelY = 0;
			goal_base.MoveX = 0;
			goal_base.MoveY = 0;
			goal_base.ThetaDeg = yaw_goal_rad;
			
			ROS_INFO("ALIGNED x: %.3f, z: %.3f, theta: %.3f\n", goal_base.MoveX  , goal_base.MoveY , yaw_goal_rad); 
		
			ac_base.sendGoal(goal_base);
			usleep(1000* 5 * 1000); 
			
			orientation_aligned_finished = 1;
			
			goal_base.wheelmove_cmd = WHEEL_MOVE_STOP;
			goal_base.VelX = 0;
			goal_base.VelY = 0;
			ROS_INFO("STOPPING orientation movement");
			ac_base.sendGoal(goal_base);
			
			orientation_align_start = 0;
			
		} /*
        
        /* align using aruco marker */
        
        if(move_flag == 0 && marker_detected == 1 && orientation_align_start == 1)
        {
			
			if(have_aligned == 0)
			{
				ROS_INFO("align robot!");
				usleep(1000* 3 * 1000); 
				
				
				goal_base.wheelmove_cmd = WHEEL_MOVE_START;
				goal_base.MoveX = marker_robot_x;
				goal_base.MoveY = marker_robot_y;
				//goal_base.ThetaDeg = pitch_c;
				
				ROS_INFO("ALIGNED x: %.3f, z: %.3f, theta: %.3f, thetaDeg: %.3f\n", marker_robot_x , marker_robot_y, pitch_c, pitch_c*R2Df); 
            
				ac_base.sendGoal(goal_base);
            
				have_aligned = 1;
				orientation_align_start = 0;
				
				geometry_msgs::PoseStamped arrived;
				arrived.pose.position.x = 1;
				arrived_publisher.publish(arrived);
				ROS_INFO("publishing arrived flag!"); 
				
			}
		}
        

        
        loop_rate.sleep();
    }

    //exit
    return 0;
}
