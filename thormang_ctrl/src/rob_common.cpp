#include "rob_common.h"

realrobot::realrobot()
{

    total_dof = 28; //

    dxlMode = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
    dxlTorque = 0;

    dxlModeSetClient = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("rt_dynamixel/mode");
    dxlMotorSetClient = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("rt_dynamixel/motor_set");

    dxlJointSetPub = nh.advertise<rt_dynamixel_msgs::JointSet>("rt_dynamixel/joint_set",1);
    dxlJointSub = nh.subscribe("rt_dynamixel/joint_state",1,&realrobot::JointCallback,this);

    jointCtrlSub = nh.subscribe("thormang_ctrl/joint_ctrl",1,&realrobot::UIJointCtrlCallback,this);

    smachPub = nh.advertise<std_msgs::String>("transition",1);
    smachSub = nh.subscribe("Jimin_machine/smach/container_status",1,&realrobot::SmachCallback,this);

    parameter_initialize();
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
    const static int jointID[] = {28, 27,
                     1,3,5,7,9,11,13,
                     2,4,6,8,10,12,14,
                     15,17,19,21,23,25,
                     16,18,20,22,24,26};


    for(int i=0; i<total_dof; i++)
    {
        for (int j=0; j<joint->id.size(); j++)
        {
            if(jointID[i] == joint->id[j])
            {
                q(i) = joint->angle[j];
                q_dot(i) = joint->velocity[j];
                torque(i) = joint->current[j];
            }
        }
    }
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

    const static int jointID[] = {28, 27,
                     1,3,5,7,9,11,13,
                     2,4,6,8,10,12,14,
                     15,17,19,21,23,25,
                     16,18,20,22,24,26};

    static int jointInvID[50];

    for(int i=0;i<total_dof; i++)
    {
        jointInvID[jointID[i]] = i;
    }
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

}

void realrobot::compute()
{

}

void realrobot::reflect()
{

}

void realrobot::writedevice()
{
    if(smach_state == "Power_On")
    {
        //ROS_INFO("POWER ON!");
        change_dxl_mode(rt_dynamixel_msgs::ModeSettingRequest::SETTING);
        set_torque(1);
    }
    else if (smach_state == "JointCtrl")
    {
        static int jointInvID[50];
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
