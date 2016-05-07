
#include <ros/ros.h>

#include "std_msgs/String.h"

#include "smach_msgs/SmachContainerStatus.h"
#include "smach_msgs/SmachContainerInitialStatusCmd.h"

class SMACHControl
{
private:
    ros::Subscriber smach_sub;
    ros::Publisher smach_pub;

    string state;

public:

    SMACHControl(ros::NodeHandle &nh)
    {
        smach_pub = nh.advertise<std_msgs::String>("sm_jimin/command", 1);
        smach_sub = nh.subscribe("Jimin_machine/smach/container_status", 1, &SMACHControl::smach_callback, this);
    }
    void setState(string &state)
    {
        std_msgs::String str;
        str.data = state;
        smach_pub.publish(str);
    }
    string& getState()
    {
        return state;
    }

    // Callback
    void smach_callback(smach_msgs::SmachContainerStatusConstPtr msg)
    {
        state = msg->active_states[0];
    }
};
