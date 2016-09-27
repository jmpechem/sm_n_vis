
#include "control_base.h"


const string JointName[40] = {"WaistPitch","WaistYaw",
                              "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                              "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                              "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                              "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                              "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};



int jointOccupancy[40] = {UPPER, UPPER,
                         UPPER, UPPER, UPPER, UPPER, UPPER, UPPER, UPPER,
                         UPPER, UPPER, UPPER, UPPER, UPPER, UPPER, UPPER,
                         WALKING, WALKING, WALKING, WALKING, WALKING, WALKING,
                         WALKING, WALKING, WALKING, WALKING, WALKING, WALKING,
                         HEAD, HEAD, UPPER, UPPER};

const int jointIDs[40] = {27, 28,
                          1,3,5,7,9,11,13,
                          2,4,6,8,10,12,14,
                          15,17,19,21,23,25,
                          16,18,20,22,24,26,
                          29,30,31,32};

// Constructor
controlBase::controlBase() :
    uiUpdateCount(0),
    isFirstBoot(true),
    _Init_walking_flag(false),
    _Walking_flag(false)
{

    rosrt::init();

    total_dof = 32;

    walkingCmdSub.initialize(3, nh, "thormang_ctrl/walking_cmd");
    taskCmdSub.initialize(3, nh, "thormang_ctrl/task_cmd");
    recogCmdSub.initialize(3, nh, "thormang_ctrl/recog_cmd");
    jointCtrlSub.initialize(3, nh, "thormang_ctrl/joint_ctrl");
    smachSub.initialize(3, nh, "Jimin_machine/smach/container_status");
    recogPointSub.initialize(3, nh, "custom_recog_point");

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

    for(int i=0; i<total_dof; i++)
    {
        jointStateMsgPtr->id[i] = jointID[i];
    }
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
    for (int i=0; i<28 ; i++)
        _walking_q(i) = q(i);

    if(_Init_walking_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.Init_walking_pose(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
    }
    else if (_Walking_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.compute(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
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
        _WalkingCtrl.setApproachdata(walkingCmdMsg.x,walkingCmdMsg.y,walkingCmdMsg.theta);

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
    if (smach_state == "Valve_Ready")
    {
        if(_cnt == 0)
        {
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 2.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _target_q = q;

            _index =2;
            _target_q(_index++) = -5*DEGREE;
            _target_q(_index++) = 66*DEGREE;
            _target_q(_index++) = 117*DEGREE;


            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(1.0); // duration set


            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 3.5*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _target_q = q;

            _index =5;

            _target_q(_index++) = 78*DEGREE;
            _target_q(_index++) = 9*DEGREE;
            _target_q(_index++) = 60*DEGREE;
            _target_q(_index++) = 10*DEGREE;

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(1.5); // duration set


            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 6.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _index = 2;

            _target_q = q;

            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 100*DEGREE;
            _target_q(_index++) = 90*DEGREE;
            _target_q(_index++) = -45*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 90*DEGREE;


            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(3.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }
    }
    if (smach_state == "Valve_Close")
    {
        if(_cnt == 0) {
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 5.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
        }
    }
    if (smach_state == "Door_Mission")
    {
        if(_cnt == 0) {
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 5.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
        }
    }

    if (_Joint_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.FK_compute(_upper_output_q);
        updateDesired(UPPER, _upper_output_q);
    }
    else if (_CLIK_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.IK_compute(_upper_output_q);
        updateDesired(UPPER, _upper_output_q);
    }

    _cnt++;
}

void controlBase::updateDesired(body_select body, VectorXD &update_q)
{
    for(int i=0; i<total_dof; i++)
    {
        if(jointOccupancy[i] == body)
        {
            _desired_q(i) = update_q(i);
        }
    }
}

void controlBase::UpperBodyCheckState()
{

    if(check_state_changed())
    {
        if (smach_state == "Valve_Init") // Valve Init
        {
            ROS_INFO("Valve Init");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = -45*DEGREE;
            _target_q(_index++) = 95*DEGREE;
            _target_q(_index++) = 80*DEGREE;
            _target_q(_index++) = 30*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = 65*DEGREE;
            _target_q(_index++) = 10*DEGREE;

         // L_arm
            _target_q(_index++) =  45*DEGREE;
            _target_q(_index++) = -95*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -30*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -65*DEGREE;
            _target_q(_index++) = -10*DEGREE;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
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

            _index = 2;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 100*DEGREE;
            _target_q(_index++) = 90*DEGREE;
            _target_q(_index++) = -45*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 90*DEGREE;
           // _target_q(_index++) = 0*DEGREE;

            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -100*DEGREE;
            _target_q(_index++) = -90*DEGREE;
            _target_q(_index++) = 45*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -90*DEGREE;


            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(2.0); // duration set
     
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Reach") // Valve Reach
        {
            ROS_INFO("Valve Reach");

            _target_x.resize(2,8);
            _target_x.setZero();

            Vector6D    vision_data;
            vision_data.setZero();
            vision_data(0) = recogPoint[0]; // 48.18*0.01;//+0.04;  M
            vision_data(1) = recogPoint[1]; // 17.62*0.01;//+0.02;
            vision_data(2) = recogPoint[2]; // 13.25*0.01;//+0.24;
            ROS_INFO("%f, %f, %f",recogPoint[0],recogPoint[1],recogPoint[2]);
            _target_x.row(0) << -1, vision_data(1), vision_data(2), vision_data(3), vision_data(4), vision_data(5),  5.0, 0;
            _target_x.row(1) << vision_data(0), vision_data(1), vision_data(2), 0.0, 0.0, 0.0, 5.0, 0;

            //   _target_x.row(0) <<  0, 0.07, 0.1, 5*DEGREE, 0, 0, 1.0, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)
            //  _target_x.row(1) <<  0.1, 0, 0, 0*DEGREE, 0, 0, 1.0, 0;

            if(vision_data(0) > 0.67 || vision_data(1) > 0.35 || vision_data(1) < 0.12 || vision_data(2) > 0.225 || vision_data(2) < -0.403 ){
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
                //_target_q(_index++) = 0*DEGREE;

                _UpperCtrl.SET_FK_Target(_target_q);
                _UpperCtrl.SET_FK_Parameter(5.0); // duration set

                _Joint_flag = true;
                _CLIK_flag = false;

            }
            else{
                _UpperCtrl.Set_Initialize();
                _UpperCtrl.SET_IK_Target(_target_x);
                _UpperCtrl.SET_IK_Parameter(100.0, false, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

                _Joint_flag = false;
                _CLIK_flag = true;
            }
        }
        else if (smach_state == "Valve_Close") // Valve Close
        {
            _target_q=q;
            _target_q(LA_BEGIN+6) = _target_q(LA_BEGIN+6) - 390*DEGREE;
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
        }

        // Door missions
        else if (smach_state == "Door_Init") // Door Init
        {
            ROS_INFO("Door Init");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = -45*DEGREE;
            _target_q(_index++) = 95*DEGREE;
            _target_q(_index++) = 80*DEGREE;
            _target_q(_index++) = 30*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = 65*DEGREE;
            _target_q(_index++) = 10*DEGREE;

         // L_arm
            _target_q(_index++) =  45*DEGREE;
            _target_q(_index++) = -95*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -30*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -65*DEGREE;
            _target_q(_index++) = -10*DEGREE;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
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
            cout<<_cnt<<endl;
        }

        else if (smach_state == "Door_Ready") // Door Ready
        {
            ROS_INFO("Door Ready");

               _target_q = q;

                ROS_INFO("1st motion");
                _index = RA_BEGIN;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = 80*DEGREE;
                _target_q(_index++) = 90*DEGREE;
                _target_q(_index++) = -45*DEGREE;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = 90*DEGREE;
               // _target_q(_index++) = 0*DEGREE;

                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = -80*DEGREE;
                _target_q(_index++) = -90*DEGREE;
                _target_q(_index++) = 45*DEGREE;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = -90*DEGREE;

            /*if(_cnt>5*_UpperCtrl._Hz && _cnt<10*_UpperCtrl._Hz){
                ROS_INFO("2nd motion");
                _index = RA_BEGIN;
                _target_q(_index++) = 90*DEGREE;
                _target_q(_index++) = 40*DEGREE;
                _target_q(_index++) = 45*DEGREE;
                _target_q(_index++) = 90*DEGREE;
                _target_q(_index++) = 90*DEGREE;
                _target_q(_index++) = 0*DEGREE;
                _target_q(_index++) = 90*DEGREE;
            }*/

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;

        }
	_cnt++;
    }

    if (taskCmdMsg.subtask==1)
    {
        ROS_INFO("Task Controller for arms");
        _UpperCtrl.Set_Initialize();

        _target_x.resize(1,8);
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

    if(jointCtrlMsgRecv == true)
    {
        _UpperCtrl.Set_Initialize();
        jointCtrlMsgRecv = false;
        double nowPos = q(jointInvID[jointCtrlMsg.id]);
        double aimPos = nowPos + jointCtrlMsg.angle * DEGREE;
        double run_time = fabs(jointCtrlMsg.angle)/10.0 ;

        _target_q = q;
        _target_q(jointInvID[jointCtrlMsg.id]) = aimPos;


        _UpperCtrl.SET_FK_Target(_target_q);
        _UpperCtrl.SET_FK_Parameter(run_time); // duration set

        _Joint_flag = true;
        _CLIK_flag = false;
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
    recogPointPtr = recogPointSub.poll();
    if(recogPointPtr)
    {
        for(int i=0; i<6; i++)
        {
            recogPoint[i] = recogPointPtr->data[i];
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

    UpperBodyLoop();
    WalkingLoop();

  /*  if(smach_state == "Valve_Reach" ||smach_state == "Valve_Ready" || smach_state == "Valve_Close")
    {
int suhan;
		suhan=RA_BEGIN;
	    _desired_q(suhan++) = -40*DEGREE;
            _desired_q(suhan++) = 75*DEGREE;
            _desired_q(suhan++) = 90*DEGREE;
            _desired_q(suhan++) = 35*DEGREE;
            _desired_q(suhan++) = 0*DEGREE;
            _desired_q(suhan++) = 60*DEGREE;
            _desired_q(suhan++) = 90*DEGREE;
    }*/
}

void controlBase::reflect()
{

    if(++uiUpdateCount > 4)
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
        _Joint_flag = false;
        _CLIK_flag = false;
        return true;
    }
    return false;
}

void controlBase::parameter_initialize()
{
    q.resize(total_dof); q.setZero();
    q_dot.resize(total_dof); q_dot.setZero();
    torque.resize(total_dof); torque.setZero();
    leftFootFT.setZero();  rightFootFT.setZero(); gyro.setZero();
    _desired_q.resize(total_dof); _desired_q.setZero();
    _target_q.resize(total_dof); _target_q.setZero();
    _upper_output_q.resize(total_dof); _upper_output_q.setZero();
    _walking_output_q.resize(28); _walking_output_q.setZero();
    _walking_q.resize(28); _walking_q.setZero();
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
