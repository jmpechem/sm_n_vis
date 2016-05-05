#include "rt_ros_service.h"



RTROSPublisher::RTROSPublisher(ros::NodeHandle &nh)
{
    pubState.initialize(nh.advertise<rt_dynamixel_msgs::JointState>("rt_dyanmixel/joint_state",0),
                        1, rt_dynamixel_msgs::JointState());
    rt_task_create(&rttTaskObject,"dxl ros pub",0,10,0);
    jointMsg = pubState.allocate();

    jointMsg->id.resize(nTotalMotors);
    jointMsg->angle.resize(nTotalMotors);
    jointMsg->velocity.resize(nTotalMotors);
    jointMsg->current.resize(nTotalMotors);

    int _cnt=0;
    for(int i=0;i<4;i++)
        for(int j=0;j<nDXLCount[i];j++)
        {
            jointMsg->id[_cnt++] = dxlLists[i][j].id;
        }
}

RTROSSubscriber::RTROSSubscriber(ros::NodeHandle &nh)
{
    rt_task_create(&rttSubscriber,"dxl ros sub",0,9,0);
    subSetter.initialize(3,nh,"rt_dynamixel/joint_set");
}


RTROSMotorSettingService::RTROSMotorSettingService(ros::NodeHandle &nh)
{
    rt_task_create(&rttMotorSetTask,"dxl motorset service",0,7,T_JOINABLE);
    modeServer = nh.advertiseService("rt_dynamixel/mode",&RTROSMotorSettingService::modeSwitch,this);
    motorServer = nh.advertiseService("rt_dynamixel/motor_set",&RTROSMotorSettingService::modeSwitch,this);
}

bool RTROSMotorSettingService::modeSwitch(rt_dynamixel_msgs::ModeSettingRequest &req,
                rt_dynamixel_msgs::ModeSettingResponse &res)
{

    res.result = -1;
    switch (req.mode)
    {
    case rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlLoopEnable = true;
        }
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::CONTROL_RUN;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::DISABLE:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlLoopEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::DISABLE;
        break;

    case rt_dynamixel_msgs::ModeSettingRequest::SETTING:
        for(int i=0;i<4;i++)
        {
            dxlDevice[i].bControlLoopEnable = false;
        }

        // wait for process end
        for(int i=0;i<4;i++)
            while(dxlDevice[i].bControlLoopProcessing) {}
        res.result =  rt_dynamixel_msgs::ModeSettingRequest::SETTING;

        break;

    default:
        break;
    }


    return true;
}


bool RTROSMotorSettingService::motorSet(rt_dynamixel_msgs::MotorSettingRequest &req,
                rt_dynamixel_msgs::MotorSettingResponse &res)
{
    res.result = -1;

    motorRequest = req;
    rt_task_start(&rttMotorSetTask, &motor_set_proc, (void*)this);
    rt_task_join(&rttMotorSetTask);

    res.result = req.mode;


    return true;
}

//////////////////////////////////////////////////////
/// \brief publisher_proc
/// \param arg
///
///
///

void publisher_proc(void *arg)
{
    RTROSPublisher* pObj = (RTROSPublisher*)arg;
    int i,j;

    rt_task_set_periodic(NULL, TM_NOW, 2e6);    // 1e6 -> 1ms   5e5 -> 500us

    while (1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        // Data set
        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_acquire();
        }

        int _cnt = 0;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                // SI
                pObj->jointMsg->angle[_cnt] = dxlDevice[i][j].position_rad();
                pObj->jointMsg->velocity[_cnt] = dxlDevice[i][j].velocity_radsec();
                pObj->jointMsg->current[_cnt] = dxlDevice[i][j].current_amp();
                _cnt++;
            }
        }

        for(i=0;i<4;i++)
        {
            dxlDevice[i].mutex_release();
        }

        pObj->pubState.publish(pObj->jointMsg);
    }
}



void subscribe_proc(void *arg)
{
    RTROSSubscriber* rtsub = (RTROSSubscriber*)arg;
    rt_task_set_periodic(NULL, TM_NOW, 1e6);    // 1e6 -> 1ms
    int i;
    while(1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        rt_dynamixel_msgs::JointSetConstPtr rcvMsg = rtsub->subSetter.poll();
        if(rcvMsg) // if message recieved ( if not rcvMsg == NULL )
        {
            // Data set
            for(i=0;i<4;i++)
            {
                dxlDevice[i].mutex_acquire();
            }

            for(i=0;i< (int)rcvMsg->id.size();i++)
            {
                dxl_from_id(rcvMsg->id[i]).aim_radian = rcvMsg->angle[i];
            }

            for(i=0;i<4;i++)
            {
                dxlDevice[i].mutex_release();
            }
        }
    }
}

void motor_set_proc(void *arg)
{
    RTROSMotorSettingService *pObj = (RTROSMotorSettingService*)arg;
    switch (pObj->motorRequest.mode)
    {
    case rt_dynamixel_msgs::MotorSettingRequest::SET_TORQUE_ENABLE:
        for (int i=0; i<4; i++)
        {
            dxlDevice[i].setAllTorque(pObj->motorRequest.value);
        }
        break;

    default:
        break;
    }
}