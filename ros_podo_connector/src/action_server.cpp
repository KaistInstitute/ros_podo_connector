
/*for ROS Action msg */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
/*Pre-defined Action msg for PODO motion */
#include <ros_podo_connector/RosPODOmotionAction.h>

/*custom header for ROS2PODO */
#include "ROSLANData.h"


/*for TCP/IP socket client */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define PODO_ADDR       "10.12.3.30"
#define PODO_PORT       5000
char ip[20];

int sock = 0;
struct sockaddr_in  server;

/*Buffer for Reading and Writing data*/
int     RXDataSize;
int     TXDataSize;
void*   RXBuffer;
void*   TXBuffer;

LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;


/*========================== Start of Action Server ==================================*/

/*ROS Action Server for handling PODO Motion*/
class RosPODOmotionAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ros_podo_connector::RosPODOmotionAction> as_; 
  std::string action_name_;
  
  // create messages that are used to published feedback&result
  ros_podo_connector::RosPODOmotionFeedback feedback_;
  ros_podo_connector::RosPODOmotionResult result_;
  
  //timer start
  ros::Time beginTime = ros::Time::now();

public:

  RosPODOmotionAction(std::string name) :
    as_(nh_, name, boost::bind(&RosPODOmotionAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~RosPODOmotionAction(void)
  {
  }

/*Call Back function when goal is received from Action client*/
  void executeCB(const ros_podo_connector::RosPODOmotionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    
    // publish info to the console for the user
    ROS_INFO("%s: Received Motion with Command: %i\n", action_name_.c_str(), goal->command);
    
    //=====execute action for robot motion========
    
    //write TX to podo
    
      static int rxDoneFlag = 0;
      TXData.ros2podo_data.ROS_CMD = goal->command;
	  TXData.ros2podo_data.ROS_MODE = goal->mode;
	  
	  TXData.ros2podo_data.wbik[RIGHT_HAND].ONOFF_movepos = 1;
      TXData.ros2podo_data.wbik[RIGHT_HAND].Goal_pos[0] =  0.5;
      TXData.ros2podo_data.wbik[RIGHT_HAND].Goal_pos[1] =  -0.3;
      TXData.ros2podo_data.wbik[RIGHT_HAND].Goal_pos[2] =  0.2;
      TXData.ros2podo_data.wbik[RIGHT_HAND].GoalmsTime = 2000;
      write(sock, &TXData, TXDataSize);
      
      //loop until TX complete
      while(rxDoneFlag == 0)
      {
		  // check that preempt has not been requested by the client
		  if (as_.isPreemptRequested() || !ros::ok())
		  {
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
			success = false;
			break;
		  }
		  
		  //DO Stuff
		  ros::Time curTime = ros::Time::now();
		  ros::Duration durTime = curTime - beginTime;
		  
		  


		  
		  if(durTime.toSec() > 10)
		  {
			  rxDoneFlag = 1;
		  }
		  
		  //feedback
		  feedback_.wbik_ref[0].goal_position[0] = (float)durTime.toSec();
		  
		  //publish feedback and result
		  as_.publishFeedback(feedback_);
		  
		  //maintain desired loop rate
		  r.sleep();
		  
	  }
	  
    
    
    //result 
    if(success)
    {
      result_.wbik_ref[0].goal_position[0] = feedback_.wbik_ref[0].goal_position[0] ;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
  
  void publishFeedback()
  {
	  
  }
  
  void publishResult()
  {
	  
  }
  
  
  


};
/*========================== End of Action Server ==================================*/



/*Print helpful summary about this code */
void printInitialInfo()
{
	std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Starting ROS2PODO Motion Action Server" << std::endl << std::endl;
    std::cout << "   Developer: Moonyoung Lee" << std::endl;
    std::cout << "   E-mail   : ml634@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;
}



/*input: IP, PORT #    */
/*function: standard cpp socket setting */
/*output: success/fail */
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

/*Check valid TCP/IP connection*/
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

/*Main Lan Thread to handle RX data*/
void* LANThread(void *){
	static int threadWorking = false;
	static int connectionStatus = false;
    threadWorking = true;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;


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

            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected, read data
            tcp_size = read(sock, RXBuffer, RXDataSize);
            if(tcp_size == RXDataSize){
                memcpy(&RXData, RXBuffer, RXDataSize);
                //NewRXData();
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


/*Load IP txt file for TCP/IP TX/RX */
/*Create Socket and initialize TX/RX size */
void initializeSocket()
{
	//load file
	FILE *fpNet = NULL;
    fpNet = fopen("/home/rainbow/catkin_ws/src/ros_podo_connector/ros_podo_connector/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }
    
    pthread_t LANTHREAD_t;
    
    //create socket
     if(CreateSocket(ip, PODO_PORT)){
        ROS_INFO("Created Socket..");

        RXDataSize = sizeof(LAN_PODO2ROS);
        TXDataSize = sizeof(LAN_ROS2PODO);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        
        //thread handler create
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0){
            ROS_ERROR("Create Thread Error..");
            
        }
    }else{
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node..");
        
    }
    
}

/*======================   Main Loop  =======================*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rospodomotion");
  printInitialInfo();
  initializeSocket();
  
  RosPODOmotionAction rospodo("rospodomotion");
  ROS_INFO("Starting ROS2PODO Action Server");
  ros::spin();

  return 0;
}
