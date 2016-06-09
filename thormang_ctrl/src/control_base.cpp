
#include "control_base.h"


const string JointName[28] = {"WaistPitch","WaistYaw",
                             "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                             "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                             "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                             "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};

const int jointIDs[28] = {28, 27,
                 1,3,5,7,9,11,13,
                 2,4,6,8,10,12,14,
                 15,17,19,21,23,25,
                 16,18,20,22,24,26};

// Constructor
controlBase::controlBase() :
    uiUpdateCount(0),
    isFirstBoot(true),
    _Init_walking_flag(false),
    _Walking_flag(false)
{

    rosrt::init();

    total_dof = 28;

    walkingCmdSub.initialize(3, nh, "thormang_ctrl/walking_cmd");
    taskCmdSub.initialize(3, nh, "thormang_ctrl/task_cmd");
    recogCmdSub.initialize(3, nh, "thormang_ctrl/recog_cmd");
    jointCtrlSub.initialize(3, nh, "thormang_ctrl/joint_ctrl");
    smachSub.initialize(3, nh, "Jimin_machine/smach/container_status");

    jointStateUIPub.initialize(nh, "thormang_ctrl/joint_state",1,1,thormang_ctrl_msgs::JointState());
    smachPub.initialize(nh, "transition",1,1,std_msgs::String());

    jointStateMsgPtr = jointStateUIPub.allocate();
    smachMsgPtr = smachPub.allocate();

    jointStateMsgPtr->angle.resize(total_dof);
    jointStateMsgPtr->velocity.resize(total_dof);
    jointStateMsgPtr->error.resize(total_dof);
    jointStateMsgPtr->current.resize(total_dof);
    jointStateMsgPtr->id.resize(total_dof);

    make_id_inverse_list();
    parameter_initialize();
}


void controlBase::make_id_inverse_list()
{
    jointInvID.resize(50);
    for(int i=0;i<total_dof; i++)
    {
        jointID.push_back(jointIDs[i]);
        jointInvID[jointIDs[i]] = i;
    }
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
    //// Auto Mission /////
    if (smach_state == "Valve_Close")
    {
        if(_cnt == 0) {
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 1.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
        }
    }

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

    _cnt++;
}

void controlBase::UpperBodyCheckState()
{

    if(check_state_changed())
    {
        if (smach_state == "Valve_Mission") // smach_state
        {
            ROS_INFO("Joint CTRL for UpperBody");

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

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(1.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Init") // Valve Init
        {
            ROS_INFO("Valve Init");

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
            _UpperCtrl.SET_FK_Parameter(1.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Ready") // Valve Ready
        {
            ROS_INFO("Valve Ready");

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

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(1.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Reach") // Valve Reach
        {
            ROS_INFO("Valve Reach");

            _target_x.resize(2,8);
            _target_x.setZero();
            _target_x.row(0) <<  0, 0.07, 0.1, 5*DEGREE, 0, 0, 1.0, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)
            _target_x.row(1) <<  0.1, 0, 0, 0*DEGREE, 0, 0, 1.0, 0;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Joint_flag = false;
            _CLIK_flag = true;
        }
        else if (smach_state == "Valve_Close") // Valve Close
        {
            _target_q=q;
            _target_q(LA_BEGIN+6) = _target_q(LA_BEGIN+6) - 390*DEGREE;
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(1.0); // duration set

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
        }
    }

    if (taskCmdMsg.subtask==1)
    {
            ROS_INFO("Task Controller for arms");
            _UpperCtrl.Set_Initialize();

            _target_x.resize(2,8);
            _target_x.setZero();
            double run_time = 0.0f;
            run_time = 5.0;
            std::cout << taskCmdMsg.x << taskCmdMsg.y << taskCmdMsg.z << taskCmdMsg.roll << taskCmdMsg.pitch << taskCmdMsg.yaw << run_time << taskCmdMsg.arm << std::endl;
            _target_x.row(0) <<  taskCmdMsg.x*0.01, taskCmdMsg.y*0.01, taskCmdMsg.z*0.01, taskCmdMsg.roll*DEGREE, taskCmdMsg.pitch*DEGREE, taskCmdMsg.yaw*DEGREE, run_time, taskCmdMsg.arm; // x,y,z,a,b,r,duration, Right(1) or Left(0)

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
            _Joint_flag = false;
            _CLIK_flag = true;
            taskCmdMsg.subtask=taskCmdMsg.NONE;
    }
}

void controlBase::update()
{

    walkingMsgPtr = walkingCmdSub.poll();
    if(walkingMsgPtr)   // Data recv
    {
        walkingCmdMsg = *walkingMsgPtr;
    }
    taskMsgPtr = taskCmdSub.poll();
    if(taskMsgPtr)
    {
        taskCmdMsg = *taskMsgPtr;
    }
    recogMsgPtr = recogCmdSub.poll();
    if(recogMsgPtr)
    {
        // Recog message received
    }
    jointSetMsgPtr = jointCtrlSub.poll();
    if(jointSetMsgPtr)
    {
        jointCtrlMsg = *jointSetMsgPtr;
        jointCtrlMsgRecv = true;
    }
    smachStatusMsgPtr = smachSub.poll();
    if(smachStatusMsgPtr)
    {
        smach_state = smachStatusMsgPtr->active_states[0];
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

void controlBase::reflect()
{
    if(++uiUpdateCount > 10)
    {
        uiUpdateCount = 0;
        for(int i=0; i<total_dof; i++)
        {
            jointStateMsgPtr->angle[i] = q(i);
            jointStateMsgPtr->velocity[i] = q_dot(i);
            jointStateMsgPtr->current[i] = torque(i);
        }

        jointStateUIPub.publish(jointStateMsgPtr);
    }
}
// Common functions
bool controlBase::check_state_changed()
{
    if(before_state != smach_state)
    {
        before_state = smach_state;
        _UpperCtrl.Set_Initialize();
        _cnt = 0;
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
double controlBase::Rounding( double x, int digit )
{
    return ( floor( (x) * pow( float(10), digit ) + 0.5f ) / pow( float(10), digit ) );
}


/*
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
  taskCmdMsg = *msg;
}

void controlBase::RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg)
{

}
*/
