#ifndef ROSCOMMAND_H
#define ROSCOMMAND_H

#define ROS_SHM_NAME        "ROS_SHARED_MEMORY"
enum ROS2PODO_COMMAND
{
    CMD_NOTHING = 0,
    CMD_MOVE_JOINT,
};

enum {//command
    MODE_JOINT_PUBLISH = 0,
    MODE_MOVE_JOINT,
    MODE_SET_WBIK
};

enum{
    rosRSP=0, rosRSR, rosRSY, rosREB, rosRWY, rosRWP, rosRWY2,
    rosLSP, rosLSR, rosLSY, rosLEB, rosLWY, rosLWP, rosLWY2,
    rosRHAND, rosLHAND,
    rosWST, rosRWH, rosLWH, rosBWH, NUM_JOINTS
};

enum{
    RIGHT_HAND = 0, RIGHT_ELBOW, LEFT_HAND, LEFT_ELBOW,
    PEL, WAIST, NUM_PARTS
};
enum{//move_mode
    MOVE_3DONE = 0, MOVE_PUBLISH, MOVE_JOINT, MOVE_WBIK
};

const std::string JointBufferNameList[NUM_JOINTS] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2",
    "RHAND", "LHAND",
    "WST", "RWH", "LWH", "BWH"
};

typedef struct _JOINT_DATA_
{
    int move_mode;
    double reference;
    double mstime;
}JOINT_DATA;

typedef struct _WBIK_DATA_
{
    int move_mode;
    int move_pos;
    int move_ori;
    double pos[3];
    double quat[4];
    double angle;
    double stime;
}WBIK_DATA;


typedef struct _ROS2PODO_DATA_
{
    int command;
    int mode;
    int sub_mode;

    JOINT_DATA joint[NUM_JOINTS];
    WBIK_DATA  wbik[NUM_PARTS];
}ROS2PODO_DATA;

typedef struct _ROS_SHM_
{
    ROS2PODO_DATA r2p;
}ROS_SHM, *pROS_SHM;

typedef struct _PODO2ROS_DATA_
{

}PODO2ROS_DATA;

#endif // COMMAND_H
