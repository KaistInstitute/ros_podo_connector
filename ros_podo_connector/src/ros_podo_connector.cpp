#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>

// for socket client
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "ROSLANData.h"


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define PODO_ADDR       "10.12.3.30"
#define PODO_PORT       5000

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;
char ip[20];

int sock = 0;
struct sockaddr_in  server;

pthread_t LANTHREAD_t;
int threadWorking = false;
int connectionStatus = false;


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void*);
void    NewRXData();

LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;

int     RXDataSize;
int     TXDataSize;
void*   RXBuffer;
void*   TXBuffer;


sensor_msgs::JointState joint_state;

//subscriber
ros::Subscriber joint_states_sub;


void joint_states_callback(const sensor_msgs::JointState& joint_state_msg){
   if(connectionStatus){
       std::cout<< "callback is called" << std::endl;

       //Match Joint names
       for(int i=0; i< NUM_JOINTS; i++){ //joints in "joint_state" are listed in different order with the joints in joint_information.h
           for(int j=0; j< joint_state_msg.name.size(); j++){                           //go through all the joint names of "joint_states"
               if(JointBufferNameList[i] == joint_state_msg.name[j]){  // store the joint information only when joint name matches
                    TXData.ros2podo_data.joint[i].reference = joint_state_msg.position[j];  //Joint Reference
                    TXData.ros2podo_data.joint[i].move_mode = MOVE_PUBLISH;                 //Move mode
                    //TXData.ros2podo_data.joint[i].mstime = [your_TIME];                   //Move time in ms
               }
           }
       }
       write(sock, &TXData, TXDataSize);

   }
}


int main(int argc, char **argv)
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Now Start the PODOConnetor..!!" << std::endl << std::endl;
    std::cout << "   Developer: Jeongsoo Lim" << std::endl;
    std::cout << "   E-mail   : yjs0497@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;

    ros::init(argc, argv, "ros_podo_connector");
    ros::NodeHandle n;

    joint_states_sub = n.subscribe("joint_states", 100, joint_states_callback);
    ros::Rate loop_rate(200);

    // Create Socket ---------------------
    FILE *fpNet = NULL;
    fpNet = fopen("/home/rainbow/catkin_ws/src/mobile_hubo_omniWH/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }

    if(CreateSocket(ip, PODO_PORT)){
        ROS_INFO("Created Socket..");

        RXDataSize = sizeof(LAN_PODO2ROS);
        TXDataSize = sizeof(LAN_ROS2PODO);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0){
            ROS_ERROR("Create Thread Error..");
            return 0;
        }
    }else{
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node..");
        return 0;
    }

    ros::Time tzero(0);

    //Subscribe topic "joint_states"
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
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

int Connect2Server(){
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0){
        std::cout << " Connection Failed" << std::endl;
        return false;
    }
    std::cout << "Client connect to server!! (PODO_CONNECTOR)" << std::endl;
    return true;
}

void NewRXData(){
		
    joint_state.header.stamp = ros::Time::now();

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
                NewRXData();
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
