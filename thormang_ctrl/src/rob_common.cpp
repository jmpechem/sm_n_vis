#include "rob_common.h"

realrobot::realrobot()
{
    dxlMode = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
    dxlTorque = 0;

    dxlModeSetClient = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("rt_dynamixel/mode");
    dxlMotorSetClient = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("rt_dynamixel/motor_set");

    dxlJointSetPub = nh.advertise<rt_dynamixel_msgs::JointSet>("rt_dynamixel/joint_set",1);
    dxlJointSub = nh.subscribe("rt_dynamixel/joint_state",1,&realrobot::JointCallback,this);

    //make_id_inverse_list();
}
void realrobot::JointCallback(const rt_dynamixel_msgs::JointStateConstPtr& joint)
{
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

/*
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
*/


void realrobot::update()
{
}
void realrobot::reflect()
{
    if(++uiUpdateCount > 10)
    {
        uiUpdateCount = 0;
        for(int i=0; i<total_dof; i++)
        {
            jointStateMsgPtr->id[i] = jointID[i];
            jointStateMsgPtr->angle[i] = (q(i) * 57.295791433);
            jointStateMsgPtr->velocity[i] = (q_dot(i) * 57.295791433);
            jointStateMsgPtr->current[i] = (torque(i));
        }

        jointStateUIPub.publish(jointStateMsgPtr);
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
    else
    {
        for(int i=0; i< total_dof; i++)
        {
            jointSetMsg.angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(jointSetMsg);

    }
}

