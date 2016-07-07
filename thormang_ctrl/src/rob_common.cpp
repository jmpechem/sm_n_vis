#include "rob_common.h"

realrobot::realrobot() : rate(250.0)
{
    dxlMode = rt_dynamixel_msgs::ModeSettingRequest::SETTING;
    dxlTorque = 0;

    dxlModeSetClient = nh.serviceClient<rt_dynamixel_msgs::ModeSetting>("rt_dynamixel/mode");
    dxlMotorSetClient = nh.serviceClient<rt_dynamixel_msgs::MotorSetting>("rt_dynamixel/motor_set");

    dxlJointSetPub.initialize(nh, "rt_dynamixel/joint_set", 1, 1, rt_dynamixel_msgs::JointSet());


    dxlJointSub = nh.subscribe("rt_dynamixel/joint_state", 1, &realrobot::jointCallback, this, ros::TransportHints().tcpNoDelay(true));
    imuSub = nh.subscribe("imu/imu", 1, &realrobot::imuCallback, this, ros::TransportHints().tcpNoDelay(true));
    imuFilterSub = nh.subscribe("imu/filter", 1, &realrobot::imuFilterCallback, this, ros::TransportHints().tcpNoDelay(true));

    leftFootFTSub = nh.subscribe("ati_ft_sensor/left_foot_ft", 1, &realrobot::leftFootFTCallback, this);
    rightFootFTSub = nh.subscribe("ati_ft_sensor/right_foot_ft", 1, &realrobot::rightFootFTCallback, this);
    /*
    dxlJointSub.initialize(3, nh, "rt_dynamixel/joint_state");

    imuSub.initialize(3, nh, "imu/imu");
    leftFootFTSub.initialize(3, nh, "ati_ft_sensor/left_foot_ft");
    rightFootFTSub.initialize(3, nh, "ati_ft_sensor/right_foot_ft");
*/

    dxlJointSetMsgPtr = dxlJointSetPub.allocate();

    dxlJointSetMsgPtr->angle.resize(total_dof);
    dxlJointSetMsgPtr->id.resize(total_dof);

    for(int i=0;i<total_dof; i++)
    {
        jointStateMsgPtr->id[i] = jointID[i];
        dxlJointSetMsgPtr->id[i] = jointID[i];
    }
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

    /*
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



    imuMsgPtr = imuSub.poll();
    if(imuMsgPtr)
    {
        gyro[0] = imuMsgPtr->angular_velocity.x;
        gyro[1] = imuMsgPtr->angular_velocity.y;
        gyro[2] = imuMsgPtr->angular_velocity.z;

        accelometer[0] = imuMsgPtr->linear_acceleration.x;
        accelometer[1] = imuMsgPtr->linear_acceleration.y;
        accelometer[2] = imuMsgPtr->linear_acceleration.z;
    }

    leftFootFTMsgPtr = leftFootFTSub.poll();
    if(leftFootFTMsgPtr)
    {
        leftFootFT[0] = leftFootFTMsgPtr->wrench.force.x;
        leftFootFT[1] = leftFootFTMsgPtr->wrench.force.y;
        leftFootFT[2] = leftFootFTMsgPtr->wrench.force.z;
        leftFootFT[3] = leftFootFTMsgPtr->wrench.torque.x;
        leftFootFT[4] = leftFootFTMsgPtr->wrench.torque.y;
        leftFootFT[5] = leftFootFTMsgPtr->wrench.torque.z;
    }
    rightFootFTMsgPtr = rightFootFTSub.poll();
    if(rightFootFTMsgPtr)
    {
        rightFootFT[0] = rightFootFTMsgPtr->wrench.force.x;
        rightFootFT[1] = rightFootFTMsgPtr->wrench.force.y;
        rightFootFT[2] = rightFootFTMsgPtr->wrench.force.z;
        rightFootFT[3] = rightFootFTMsgPtr->wrench.torque.x;
        rightFootFT[4] = rightFootFTMsgPtr->wrench.torque.y;
        rightFootFT[5] = rightFootFTMsgPtr->wrench.torque.z;
    }
*/
}
void realrobot::reflect()
{
    // update additional information

    for(int i=0; i<total_dof; i++)
    {
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
        for(int i=0; i< total_dof; i++)
        {
            dxlJointSetMsgPtr->angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(dxlJointSetMsgPtr);
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
    else if (smach_state == "Manual")
    {
        change_dxl_mode(rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN);
        for(int i=0; i< total_dof; i++)
        {
            dxlJointSetMsgPtr->angle[i] = _desired_q(i);
        }
        dxlJointSetPub.publish(dxlJointSetMsgPtr);
    }
    else if (smach_state == "None")
    {
        change_dxl_mode(rt_dynamixel_msgs::ModeSettingRequest::SETTING);
        set_torque(0);
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
    rate.sleep();
    /*
    static int asdf = 0;
    asdf++;
    if(asdf>=299)
    {
        ROS_INFO("1s");
        asdf=0;
    }
    */
    //ROS_INFO("LOOP");
    /*
    rtNowTime = rt_timer_read();
    while( rtNowTime < rtNextTime )
    {
        //rt_task_sleep()
        rtNowTime = rt_timer_read();
        usleep(10);
    }
    rtNextTime = rtNowTime;*/
}
