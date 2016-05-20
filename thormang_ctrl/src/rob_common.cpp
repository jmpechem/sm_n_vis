#include "rob_common.h"

realrobot::realrobot() : uiUpdateCount(0), isFirstBoot(true)
{

    total_dof = 28; //

    dxlMode = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
    dxlTorque = 0;

    dxlModeSetClient = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("rt_dynamixel/mode");
    dxlMotorSetClient = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("rt_dynamixel/motor_set");

    dxlJointSetPub = nh.advertise<rt_dynamixel_msgs::JointSet>("rt_dynamixel/joint_set",1);
    dxlJointSub = nh.subscribe("rt_dynamixel/joint_state",1,&realrobot::JointCallback,this);

    jointStateUIPub = nh.advertise<thormang_ctrl_msgs::JointState>("thormang_ctrl/joint_state",1);
    jointCtrlSub = nh.subscribe("thormang_ctrl/joint_ctrl",1,&realrobot::UIJointCtrlCallback,this);

    smachPub = nh.advertise<std_msgs::String>("transition",1);
    smachSub = nh.subscribe("Jimin_machine/smach/container_status",1,&realrobot::SmachCallback,this);

    parameter_initialize();
    make_id_inverse_list();
}

void realrobot::UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr &joint)
{
    jointCtrlMsg = *joint;
    jointCtrlMsgRecv = true;
}

void realrobot::JointCallback(const rt_dynamixel_msgs::JointStateConstPtr& joint)
{
    /*
    string JointName[] = {"WaistPitch","WaistYaw",
                             "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                             "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                             "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                             "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};
    */

    for(int i=0; i<total_dof; i++)
    {
        for (int j=0; j<joint->id.size(); j++)
        {
            if(jointID[i] == joint->id[j])
            {
                q(i) = joint->angle[j];
                if(isFirstBoot)
                {    _desired_q(i) = joint->angle[j]; }

                q_dot(i) = joint->velocity[j];
                torque(i) = joint->current[j];
            }
        }
    }
    if(isFirstBoot)
    {isFirstBoot = false;}
}

void realrobot::SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach)
{
    smach_state = smach->active_states[0];
}

void realrobot::change_dxl_mode(int mode)
{
    if(dxlMode != mode) // mode is changed
    {
        rt_dynamixel_msgs::ModeSettingRequest req;
        rt_dynamixel_msgs::ModeSettingResponse res;

        req.mode = mode;

        dxlModeSetClient.call(req,res);
        dxlMode = mode;

    }
}

void realrobot::set_aim_position(int id, double radian)
{
    if(dxlMode == rt_dynamixel_msgs::ModeSettingRequest::SETTING) // Setting mode?
    {
        rt_dynamixel_msgs::MotorSettingRequest req;
        rt_dynamixel_msgs::MotorSettingResponse res;

        req.mode=rt_dynamixel_msgs::MotorSettingRequest::SET_GOAL_POSITION;
        req.id = id;
        req.fvalue = radian;

        dxlMotorSetClient.call(req,res);
    }

}

void realrobot::set_torque(int value)
{
    if(dxlMode == rt_dynamixel_msgs::ModeSettingRequest::SETTING) // Setting mode?
        if(dxlTorque != value)  // value is changed
        {
            rt_dynamixel_msgs::MotorSettingRequest req;
            rt_dynamixel_msgs::MotorSettingResponse res;

            req.mode=rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE;
            req.value = value;

            dxlMotorSetClient.call(req,res);
            dxlTorque = value;
        }
}

void realrobot::make_id_inverse_list()
{

    const static int jointIDs[] = {28, 27,
                     1,3,5,7,9,11,13,
                     2,4,6,8,10,12,14,
                     15,17,19,21,23,25,
                     16,18,20,22,24,26};


    jointInvID.resize(50);
    for(int i=0;i<total_dof; i++)
    {
        jointID.push_back(jointIDs[i]);
        jointSetMsg.id.push_back(jointIDs[i]);
        jointInvID[jointIDs[i]] = i;
    }
    jointSetMsg.angle.resize(total_dof);
}

void realrobot::parameter_initialize()
{
    q.resize(total_dof); q.setZero();
    q_dot.resize(total_dof); q_dot.setZero();
    torque.resize(total_dof); torque.setZero();
    LFT.setZero();  RFT.setZero(); Gyro.setZero();
    _desired_q.resize(total_dof); _desired_q.setZero();
}

void realrobot::readdevice()
{
    ros::spinOnce();
}

void realrobot::update()
{
    key_cmd = getch();
}

void realrobot::compute()
{
    if (key_cmd == 'i')
    {
        ROS_INFO("Init Walking");
        _Init_walking_flag = true;
        _Walking_flag = true;
        _WalkingCtrl._initialize();
    }
    else if (key_cmd == 'w')
    {
        ROS_INFO("Walking Command");
        _Walking_flag = true;
        _Init_walking_flag = false;
        _WalkingCtrl._initialize();
    }
    else if (key_cmd == 'q')
    {
       ROS_INFO("q trigger");
  // vrep_stop();
    }

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

void realrobot::reflect()
{
    if(++uiUpdateCount > 10)
    {
        uiUpdateCount = 0;
        thormang_ctrl_msgs::JointState msg;
        for(int i=0; i<total_dof; i++)
        {
            msg.id.push_back(jointID[i]);
            msg.angle.push_back(q(i) * 57.295791433);
            msg.velocity.push_back(q_dot(i) * 57.295791433);
            msg.current.push_back(torque(i));
        }

        jointStateUIPub.publish(msg);
    }
}

void realrobot::writedevice()
{
    if(smach_state == "Power_On")
    {
        //ROS_INFO("POWER ON!");
        change_dxl_mode(rt_dynamixel_msgs::ModeSettingRequest::SETTING);
        set_torque(1);
    }
    else if (smach_state == "Auto")
    {
        change_dxl_mode(rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN);
        for(int i=0; i< total_dof; i++)
        {
            jointSetMsg.angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(jointSetMsg);
    }
    else if (smach_state == "JointCtrl")
    {
        if(jointCtrlMsgRecv == true)
        {
            jointCtrlMsgRecv = false;
            double nowPos = q(jointInvID[jointCtrlMsg.id]);
            double aimPos = nowPos + jointCtrlMsg.angle * DEGREE;
            ROS_INFO("Joint Shoot!");
            set_aim_position(jointCtrlMsg.id,aimPos);
        }
    }
}


int realrobot::getch()
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
