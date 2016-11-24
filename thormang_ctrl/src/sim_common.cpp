#include "sim_common.h"

simulation::simulation(ros::NodeHandle &nh): controlBase(nh), rate(200)
{

    simulationRunning = true;
    simulationTime=0.0f; // set initial simulation time
    subInfo = nh.subscribe("/vrep/info",100,&simulation::infoCallback,this);
    vrepHandleClient = nh.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
    client_startsync = nh.serviceClient<vrep_common::simRosSynchronous>("vrep/simRosSynchronous");
    srv_startsync.request.enable = 1;
    client_startTrig = nh.serviceClient<vrep_common::simRosSynchronousTrigger>("vrep/simRosSynchronousTrigger");

    start_simulation = nh.serviceClient<vrep_common::simRosStartSimulation>("vrep/simRosStartSimulation");
    end_simulation = nh.serviceClient<vrep_common::simRosStopSimulation>("vrep/simRosStopSimulation");

    Jointsub = nh.subscribe("vrep/JointState",100,&simulation::JointCallback,this);
    Lftsub = nh.subscribe("vrep/LFT",100,&simulation::LftCallback,this);
    Rftsub = nh.subscribe("vrep/RFT",100,&simulation::RftCallback,this);
    vrepJointSetPub = nh.advertise<vrep_common::JointSetStateData>("vrep/JointSet",100);

    vrep_start();
    vrep_initialize();
}

// V-Rep Functions
void simulation::vrep_initialize()
{     
    vrep_common::JointSetStateData data;
    data.values.data.resize(total_dof);
    vrep_common::simRosGetObjectHandle srv;
    for (int i=0; i<total_dof; i++)
    {
        srv.request.objectName = JointName[i];
        std::cout << srv.request.objectName << std::endl;
        if (vrepHandleClient.call(srv))
        {
            data.handles.data.push_back(srv.response.handle);
            data.setModes.data.push_back(1);
            data.values.data[i] = 0.0*DEGREE;
        }
    }
    JointSetValue = data;
    client_startsync.call(srv_startsync);

}
void simulation::vrep_end()
{  
    srv_startsync.request.enable = 0;
    client_startsync.call(srv_startsync);


    ROS_INFO("end of Simulation end trigger");
}
void simulation::vrep_start()
{
    vrep_common::simRosStartSimulation start;
    start_simulation.call(start);

}
void simulation::vrep_stop()
{
    vrep_common::simRosStopSimulation stop;
    end_simulation.call(stop);

}


// Function implement
void simulation::update()
{
    controlBase::update();
    key_cmd = getch();

}
void simulation::compute()
{
    if (walkingCmdMsg.command == "stop")
    {
        vrep_stop();
    }
    controlBase::compute();
}

void simulation::writedevice()
{
    for(int i=0;i<total_dof;i++)
        JointSetValue.values.data[i] = _desired_q(i);

    vrepJointSetPub.publish(JointSetValue);
    client_startTrig.call(srv_startTrig);

}

void simulation::wait()
{
    rate.sleep();
}


// Callback functions
void simulation::infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
    simulationTime=info->simulationTime.data;
    simulationRunning=(info->simulatorState.data&1)!=0;
    // cout << simulationRunning << endl;
}
void simulation::JointCallback(const sensor_msgs::JointState::ConstPtr& joint)
{
    for(int i=0; i<total_dof; i++)
    {
        string target_joint = JointName[i];
        for (int j=0; j<joint->name.size(); j++)
        {
            string joint_name = joint->name[j].data();
            if(target_joint == joint_name)
            {
                q(i) = joint->position[j];
                q_dot(i) = joint->velocity[j];
                torque(i) = joint->effort[j];

            }
        }
    }
}
void simulation::LftCallback(const vrep_common::ForceSensorData::ConstPtr& Lft)
{
    leftFootFT(0) = Lft->force.x;
    leftFootFT(1) = Lft->force.y;
    leftFootFT(2) = Lft->force.z;
    leftFootFT(3) = -Lft->torque.x;
    leftFootFT(4) = Lft->torque.y;
    leftFootFT(5) = Lft->torque.z;
}


void simulation::RftCallback(const vrep_common::ForceSensorData::ConstPtr& Rft)
{
    rightFootFT(0) = Rft->force.x;
    rightFootFT(1) = Rft->force.y;
    rightFootFT(2) = Rft->force.z;
    rightFootFT(3) = -Rft->torque.x;
    rightFootFT(4) = Rft->torque.y;
    rightFootFT(5) = Rft->torque.z;
}

