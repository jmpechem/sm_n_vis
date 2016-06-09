#include "rob_common.h"

realrobot::realrobot()
{
    dxlMode = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
    dxlTorque = 0;

    dxlModeSetClient = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("rt_dynamixel/mode");
    dxlMotorSetClient = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("rt_dynamixel/motor_set");

    dxlJointSetPub.initialize(nh, "rt_dynamixel/joint_set", 1, 1, rt_dynamixel_msgs::JointSet());
    dxlJointSub.initialize(3, nh, "rt_dynamixel/joint_state");

    dxlJointSetMsgPtr = dxlJointSetPub.allocate();

    dxlJointSetMsgPtr->angle.resize(total_dof);
    dxlJointSetMsgPtr->id.resize(total_dof);

    for(int i=0;i<total_dof; i++)
    {
        jointStateMsgPtr->id[i] = jointID[i];
        dxlJointSetMsgPtr->id[i] = jointID[i];
    }
    rtNextTime = rt_timer_read() + 3e6; // 3ms

    //make_id_inverse_list();
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

void realrobot::update()
{
    controlBase::update();
    dxlJointStatePtr = dxlJointSub.poll();
    if(dxlJointStatePtr)
    {

        for(int i=0; i<total_dof; i++)
        {
            for (int j=0; j<dxlJointStatePtr->id.size(); j++)
            {
                if(jointID[i] == dxlJointStatePtr->id[j])
                {
                    q(i) = dxlJointStatePtr->angle[j];
                    if(isFirstBoot)
                    {    _desired_q(i) = dxlJointStatePtr->angle[j]; }

                    q_dot(i) = dxlJointStatePtr->velocity[j];
                    torque(i) = dxlJointStatePtr->current[j];
                }
            }
        }
        if(isFirstBoot)
        {isFirstBoot = false;}
    }

}
void realrobot::reflect()
{
    // update additional information
    for(int i=0; i<total_dof; i++)
    {
        jointStateMsgPtr->error[i] = dxlJointStatePtr->updated[i];
    }

    controlBase::reflect();
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
            dxlJointSetMsgPtr->angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(dxlJointSetMsgPtr);
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
            dxlJointSetMsgPtr->angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(dxlJointSetMsgPtr);

    }
}

void realrobot::wait()
{
    rtNowTime = rt_timer_read();
    while( rtNowTime < rtNextTime )
    {
        //rt_task_sleep()
        rtNowTime = rt_timer_read();
    }
    rtNextTime = rtNowTime;
}
