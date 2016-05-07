
#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>

#include "rt_dynamixel_pro.h"
#include "rt_dynamixel_msgs/JointState.h"
#include "rt_dynamixel_msgs/JointSet.h"
#include "rt_dynamixel_msgs/ModeSetting.h"
#include "rt_dynamixel_msgs/MotorSetting.h"

#include "rt_ros_service.h"

#include "dxl_lists.h"

using namespace DXL_PRO;

// control period; 1e6 -> 1ms   25e5 -> 2.5ms, 400Hz
RTIME control_period = 25e5;

/**
 * @brief main proc
 * @return err=-1 done=0
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rt_dynamixel_node");
    rosrt::init();

    ros::NodeHandle nh;

    // Realtime layer communication initailization
    if(dxl_initailize() == false) return -1;

    // Get initial pose and set default settings
    dynamixel_motor_init();

    // Enable ROS Service, Pub and Sub
    RTROSPublisher rtRosPublisher(nh);
    RTROSSubscriber rtRosSubscriber(nh);
    RTROSMotorSettingService rtRosMotorSettingService(nh);

    // Start
    rtRosPublisher.start();
    rtRosSubscriber.start();

    ros::spin();

    return 0;
}







/*
void motion_init_proc(void *arg)
{
    int i,j;

    rt_task_set_periodic(NULL, TM_NOW, 250e6);    // 1e6 -> 1ms   250ms

    for(i=0;i<4;i++)
    {
        dxlDevice[i].setReturnDelayTime(0);
        dxlDevice[i].setAllAcceleration(30);
        dxlDevice[i].setAllVelocity(1000);
        dxlDevice[i].setAllTorque(1);
    }



    while(1)
    {
        rt_task_wait_period(NULL); //wait for next cycle

        for(i=0;i<4;i++)
        {
            dxlDevice[i].setAllDegree(0.0);
            dxlDevice[i].rttLoopStartTime = rt_timer_read();
            dxlDevice[i].rttLoopTimeoutTime = dxlDevice[i].rttLoopStartTime + 1e7;
            dxlDevice[i].getAllStatus();
        }

        printf("[INFO] Position List \n");

        bool isDone = true;
        for(i=0;i<4;i++)
        {
            for(j=0;j<nDXLCount[i];j++)
            {
                if(abs(dxlDevice[i][j].position) > 200) isDone = false;
                printf("%d\t", dxlDevice[i][j].position);
                printf("\n");
            }

        }
        if(isDone == true)
        {
            printf("[INFO] Motor Init Complete \n");
            break;
        }

    }

    rt_task_sleep(3000e6);

    for(i=0;i<4;i++)
    {
        dxlDevice[i].setAllAcceleration(0);
        dxlDevice[i].setAllVelocity(0);
    }
}
*/
