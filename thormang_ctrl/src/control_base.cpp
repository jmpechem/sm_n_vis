
#include "control_base.h"

// Constructor
controlBase::controlBase() :
    uiUpdateCount(0),
    isFirstBoot(true),
    _Init_walking_flag(false),
    _Walking_flag(false)
{
    total_dof = 28;

    walkingCmdSub = nh.subscribe("thormang_ctrl/walking_cmd",1,&controlBase::WalkingCmdCallback,this);
    taskCmdSub = nh.subscribe("thormang_ctrl/task_cmd",1,&controlBase::TaskCmdCallback,this);
    recogCmdSub = nh.subscribe("thormang_ctrl/recog_cmd",1,&controlBase::RecogCmdCallback,this);

    jointStateUIPub = nh.advertise<thormang_ctrl_msgs::JointState>("thormang_ctrl/joint_state",1);
    jointCtrlSub = nh.subscribe("thormang_ctrl/joint_ctrl",1,&controlBase::UIJointCtrlCallback,this);

    smachPub = nh.advertise<std_msgs::String>("transition",1);
    smachSub = nh.subscribe("Jimin_machine/smach/container_status",1,&controlBase::SmachCallback,this);

    parameter_initialize();
}

void controlBase::WalkingLoop()
{
    if(_Init_walking_flag == true)
    {
        _WalkingCtrl.getdata(q,LFT,RFT,Gyro);
        _WalkingCtrl.Init_walking_pose(_desired_q);
    }
    else if (_Walking_flag == true)
    {
        _WalkingCtrl.getdata(q,LFT,RFT,Gyro);
        _WalkingCtrl.compute(_desired_q);
    }
}

void controlBase::WalkingCheckState()
{

    if (walkingCmdMsg.command == "init")
    {
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Init");
        _Init_walking_flag = true;
        _Walking_flag = true;
        _WalkingCtrl._initialize();
    }
    else if (walkingCmdMsg.command == "start")
    {
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Start");
        _Walking_flag = true;
        _Init_walking_flag = false;
        _WalkingCtrl._initialize();
    }
    else if (walkingCmdMsg.command == "stop")
    {
        _Walking_flag = false;
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Stop");
    }
}


void controlBase::UpperBodyLoop()
{
    if (_Joint_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.FK_compute(_desired_q);
    }
    else if (_CLIK_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.IK_compute(_desired_q);
    }
}

void controlBase::UpperBodyCheckState()
{

    if(check_state_changed())
    {
        if (smach_state == "Valve_Mission") // smach_state
        {
            ROS_INFO("Joint CTRL for UpperBody");
            _UpperCtrl.Set_Initialize();

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 75*DEGREE;
            _target_q(_index++) = 90*DEGREE;
            _target_q(_index++) = 35*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 60*DEGREE;
            _target_q(_index++) = 90*DEGREE;

            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -75*DEGREE;
            _target_q(_index++) = -90*DEGREE;
            _target_q(_index++) = -35*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -60*DEGREE;
            _target_q(_index++) = -90*DEGREE;

            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 25*DEGREE;
            _target_q(_index++) = -50*DEGREE;
            _target_q(_index++) = 25*DEGREE;
            _target_q(_index++) = 2*DEGREE;

            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -25*DEGREE;
            _target_q(_index++) = 50*DEGREE;
            _target_q(_index++) = -25*DEGREE;
            _target_q(_index++) = -2*DEGREE;

            //_UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (key_cmd == 's')
        {
            ROS_INFO("CLIK CTRL for UpperBody (Singularity)");
            _UpperCtrl.Set_Initialize();

            _target_x.resize(2,8);
            _target_x.setZero();
            _target_x.row(0) <<  0, 0, 0.3, 0*DEGREE, 0, 0, 1.0, 1; // x,y,z,a,b,r,duration, Right(1) or Left(0)
            _target_x.row(1) <<  0.0, 0, 0.3,  0*DEGREE, 0, 0, 1.0, 1;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.03, 0.01); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Joint_flag = false;
            _CLIK_flag = true;
        }
        else if (smach_state == "Valve_Init") // Valve Init
        {
            ROS_INFO("Valve Init");
            _UpperCtrl.Set_Initialize();

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 75*DEGREE;
            _target_q(_index++) = 90*DEGREE;
            _target_q(_index++) = 35*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 60*DEGREE;
            _target_q(_index++) = 90*DEGREE;

            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -75*DEGREE;
            _target_q(_index++) = -90*DEGREE;
            _target_q(_index++) = -35*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -60*DEGREE;
            _target_q(_index++) = -90*DEGREE;

            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;

            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Ready") // Valve Ready
        {
            ROS_INFO("Valve Ready");
            _UpperCtrl.Set_Initialize();

            _target_q = q;

            _index = LA_BEGIN;
            _target_q(_index++) = 30*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -90*DEGREE;
            _target_q(_index++) = -100*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 180*DEGREE;
            _target_q(_index++) = 0*DEGREE;

            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Reach") // Valve Reach
        {
            ROS_INFO("Valve Reach");
            _UpperCtrl.Set_Initialize();

            _target_x.resize(2,8);
            _target_x.setZero();
            _target_x.row(0) <<  0, 0.07, 0.1, 5*DEGREE, 0, 0, 5.0, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)
            _target_x.row(1) <<  0.1, 0, 0, 0*DEGREE, 0, 0, 5.0, 0;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Joint_flag = false;
            _CLIK_flag = true;
        }
        else if (smach_state == "Valve_Close") // Valve Close
        {

        }
        else if (key_cmd == 'b') // Left Hand - 1cm up(z)
        {
            ROS_INFO("Left Hand Up");
            _UpperCtrl.Set_Initialize();

            _target_x.resize(2,8);
            _target_x.setZero();
            _target_x.row(0) <<  0, 0, 0.02, 0, 0, 0, 0.5, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Joint_flag = false;
            _CLIK_flag = true;
        }
        else if (key_cmd == 'n') // Left Hand - 1cm down(z)
        {
            ROS_INFO("Left Hand down");
            _UpperCtrl.Set_Initialize();

            _target_x.resize(2,8);
            _target_x.setZero();
            _target_x.row(0) <<  0, 0, -0.02, 0, 0, 0, 0.5, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Joint_flag = false;
            _CLIK_flag = true;
        }

    }
}

void controlBase::compute()
{
    // Update
    WalkingCheckState();
    UpperBodyCheckState();


    // Planning


    // Loop
    WalkingLoop();
    UpperBodyLoop();
}

// Common functions
bool controlBase::check_state_changed()
{
    if(before_state != smach_state)
    {
        before_state = smach_state;
        return true;
    }
    return false;
}

void controlBase::parameter_initialize()
{
    q.resize(total_dof); q.setZero();
    q_dot.resize(total_dof); q_dot.setZero();
    torque.resize(total_dof); torque.setZero();
    LFT.setZero();  RFT.setZero(); Gyro.setZero();
    _desired_q.resize(total_dof); _desired_q.setZero();
    _target_q.resize(total_dof); _target_q.setZero();
}
void controlBase::readdevice()
{
    ros::spinOnce();
}

int controlBase::getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 100;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
    { }
    else if(rv == 0)
    {}
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}



// Callback
void controlBase::UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr &joint)
{
    jointCtrlMsg = *joint;
    jointCtrlMsgRecv = true;
}

void controlBase::SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach)
{
    smach_state = smach->active_states[0];
}

void controlBase::WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg)
{
    walkingCmdMsg = *msg;
}

void controlBase::TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg)
{

}

void controlBase::RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg)
{

}

