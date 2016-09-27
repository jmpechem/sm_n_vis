#include "rob_common.h"

realrobot::realrobot() : rate(200.0)
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
    nowTime = ros::Time::now();
    uint32_t dt = nowTime.nsec - befoTime.nsec;

    if(abs(dt - 5e6) > 3e6)
    {
        double ms = dt / 1000000.0;
        ROS_WARN("realtime broken. dt = %.2lf ms", ms);
    }
    befoTime = nowTime;
*/

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



// private



void realrobot::jointCallback(const rt_dynamixel_msgs::JointStateConstPtr msg)
{

    for(int i=0; i<total_dof; i++)
    {
        for (int j=0; j<msg->id.size(); j++)
        {
            if(jointID[i] == msg->id[j])
            {
                q(i) = msg->angle[j];
                if(isFirstBoot)
                {    _desired_q(i) = msg->angle[j]; }

                q_dot(i) = msg->velocity[j];
                torque(i) = msg->current[j];
                jointStateMsgPtr->error[i] = msg->updated[j];
            }
        }
    }
    if(isFirstBoot)
    {isFirstBoot = false;}
    _jointRecv = true;
}
void realrobot::imuCallback(const sensor_msgs::ImuConstPtr msg)
{

    double rcvDatas[6];
    rcvDatas[0] = msg->linear_acceleration.x;
    rcvDatas[1] = msg->linear_acceleration.y;
    rcvDatas[2] = msg->linear_acceleration.z;

    rcvDatas[3] = msg->angular_velocity.x;
    rcvDatas[4] = msg->angular_velocity.y;
    rcvDatas[5] = msg->angular_velocity.z;

    AngleComplementaryFilter(0.004, 0.1, gyro[1], gyro[0], rcvDatas, gyro[1], gyro[0]);
    //ROS_INFO("r=%.2lf p=%.2lf",gyro[0]* 57.295791433, gyro[1]* 57.295791433);
    // ROS_INFO("roll = %.2lf, pitch = %.2lf", gyro[0] * 57.295791433, gyro[1] * 57.295791433);
    /*
    gyro[0] = msg->angular_velocity.x;
    gyro[1] = msg->angular_velocity.y;
    gyro[2] = msg->angular_velocity.z;

    accelometer[0] = msg->linear_acceleration.x;
    accelometer[1] = msg->linear_acceleration.y;
    accelometer[2] = msg->linear_acceleration.z;
    */
    _imuRecv = true;
}

void realrobot::imuFilterCallback(const imu_3dm_gx4::FilterOutputConstPtr msg)
{

    tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf::Matrix3x3 m(q);

    // m.getRPY(gyro[0],gyro[1],gyro[2]);

    // ROS_INFO("r=%.2lf p=%.2lf y=%.2lf",gyro[0]* 57.295791433, gyro[1]* 57.295791433, gyro[2]* 57.295791433);
}

void realrobot::AngleComplementaryFilter(double dt, double cutoff_freq, double pitch_i, double roll_i, double SensorData[6], double &pitch, double &roll)
{
 //sensor data = acc x, y, z, ang rate pitch, roll, yaw
 //double cutoff_freq = 0.1; //recommand value:0.1~0.2
 float wn = 2.0*3.1415*cutoff_freq;
 float alpha_const = (1.0/wn/dt)/(1.0+1.0/wn/dt);

 float pitch_Acc, roll_Acc;

 pitch_Acc = (atan2(SensorData[0],SensorData[2]));
 pitch = (alpha_const*(pitch_i-SensorData[4]*dt) + (1.0-alpha_const)*pitch_Acc);

 roll_Acc = -(atan2(SensorData[1],SensorData[2]));
 roll = (alpha_const*(roll_i-SensorData[3]*dt) + (1.0-alpha_const)*roll_Acc);
}




void realrobot::leftFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
{
    leftFootFT[0] = msg->wrench.force.x;
    leftFootFT[1] = msg->wrench.force.y;
    leftFootFT[2] = msg->wrench.force.z;
    leftFootFT[3] = msg->wrench.torque.x;
    leftFootFT[4] = msg->wrench.torque.y;
    leftFootFT[5] = msg->wrench.torque.z;
    _ftlfRecv = true;


}
void realrobot::rightFootFTCallback(const geometry_msgs::WrenchStampedConstPtr msg)
{
    rightFootFT[0] = msg->wrench.force.x;
    rightFootFT[1] = msg->wrench.force.y;
    rightFootFT[2] = msg->wrench.force.z;
    rightFootFT[3] = msg->wrench.torque.x;
    rightFootFT[4] = msg->wrench.torque.y;
    rightFootFT[5] = msg->wrench.torque.z;
    _ftrfRecv = true;
}
