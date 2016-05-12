#include "sim_common.h"

simulation::simulation(){   
   total_dof = 28; // 
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
   
}
void simulation::vrep_initialize()
{     
 string JointName[] = {"WaistPitch","WaistYaw",
                         "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                         "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                         "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                         "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};

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
void simulation::infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
    simulationTime=info->simulationTime.data;
    simulationRunning=(info->simulatorState.data&1)!=0;
   // cout << simulationRunning << endl;
}

void simulation::JointCallback(const sensor_msgs::JointState::ConstPtr& joint)
{    
string JointName[] = {"WaistPitch","WaistYaw",
                         "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                         "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                         "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                         "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll"};

  
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
    LFT(0) = Lft->force.x;
    LFT(1) = Lft->force.y;
    LFT(2) = Lft->force.z;
    LFT(3) = Lft->torque.x;
    LFT(4) = Lft->torque.y;
    LFT(5) = Lft->torque.z;
}


void simulation::RftCallback(const vrep_common::ForceSensorData::ConstPtr& Rft)
{
    RFT(0) = Rft->force.x;
    RFT(1) = Rft->force.y;
    RFT(2) = Rft->force.z;
    RFT(3) = Rft->torque.x;
    RFT(4) = Rft->torque.y;
    RFT(5) = Rft->torque.z;
}
void simulation::parameter_initialize()
{
  q.resize(total_dof); q.setZero();
  q_dot.resize(total_dof); q_dot.setZero();
  torque.resize(total_dof); torque.setZero();
  LFT.setZero();  RFT.setZero(); Gyro.setZero();
  _desired_q.resize(total_dof); _desired_q.setZero();
}

void simulation::readdevice()
{
	ros::spinOnce();
}
void simulation::update() 
{
	key_cmd = getch();
}
void simulation::compute()
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
	 vrep_stop();
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
void simulation::reflect() // publish statemachine state and else
{

}
void simulation::writedevice()
{
  for(int i=0;i<total_dof;i++)
     JointSetValue.values.data[i] = _desired_q(i);

  vrepJointSetPub.publish(JointSetValue);
  client_startTrig.call(srv_startTrig);
  
}

int simulation::getch()
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
