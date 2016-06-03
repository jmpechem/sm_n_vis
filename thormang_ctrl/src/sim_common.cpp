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

   smachPub = nh.advertise<std_msgs::String>("transition",1);
   smachSub = nh.subscribe("Jimin_machine/smach/container_status",1,&simulation::SmachCallback,this);
   
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
void simulation::SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach)
{
    smach_state = smach->active_states[0];
}
bool simulation::check_state_changed()
{
  if(before_state != smach_state)
    {
      before_state = smach_state;
      return true;
    }
  return false;
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
  _target_q.resize(total_dof); _target_q.setZero();
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
/// CLIK Controller /////////////////
      if(check_state_changed())
        {

          if (smach_state == "Valve_Mission") // smach_state
          {
             ROS_INFO("Joint CTRL for UpperBody");
             _UpperCtrl.Set_Initialize();

             _index = 0;
             _target_q(_index++) = 0;
             _target_q(_index++) = 0;
             _target_q(_index++) = -40*DEGREE;
             _target_q(_index++) = 75*DEGREE;
             _target_q(_index++) = 90*DEGREE;
             _target_q(_index++) = 35*DEGREE;
             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = 60*DEGREE;
             _target_q(_index++) = 90*DEGREE;

             _target_q(_index++) = 40*DEGREE;
             _target_q(_index++) = -75*DEGREE;
             _target_q(_index++) = -90*DEGREE;
             _target_q(_index++) = -35*DEGREE;
             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = -60*DEGREE;
             _target_q(_index++) = -90*DEGREE;

             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = -2*DEGREE;
             _target_q(_index++) = 25*DEGREE;
             _target_q(_index++) = -50*DEGREE;
             _target_q(_index++) = 25*DEGREE;
             _target_q(_index++) = 2*DEGREE;

             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = 2*DEGREE;
             _target_q(_index++) = -25*DEGREE;
             _target_q(_index++) = 50*DEGREE;
             _target_q(_index++) = -25*DEGREE;
             _target_q(_index++) = -2*DEGREE;

             //_UpperCtrl.Set_Initialize();

             _UpperCtrl.SET_FK_Target(_target_q);
             _UpperCtrl.SET_FK_Parameter(1.0); // duration set

             _Joint_flag = true;
             _CLIK_flag = false;
          }
          else if (key_cmd == 's')
          {
             ROS_INFO("CLIK CTRL for UpperBody (Singularity)");
             _UpperCtrl.Set_Initialize();

             _target_x.resize(2,8);
             _target_x.setZero();
             _target_x.row(0) <<  0, 0, 0.3, 0*DEGREE, 0, 0, 1.0, 1; // x,y,z,a,b,r,duration, Right(1) or Left(0)
             _target_x.row(1) <<  0.0, 0, 0.3,  0*DEGREE, 0, 0, 1.0, 1;

             _UpperCtrl.SET_IK_Target(_target_x);
             _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.03, 0.01); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

             _Joint_flag = false;
             _CLIK_flag = true;
          }
          else if (smach_state == "Valve_Init") // Valve Init
             {
             ROS_INFO("Valve Init");
             _UpperCtrl.Set_Initialize();

             _index = 0;
             _target_q(_index++) = 0;
             _target_q(_index++) = 0;
             _target_q(_index++) = -40*DEGREE;
             _target_q(_index++) = 75*DEGREE;
             _target_q(_index++) = 90*DEGREE;
             _target_q(_index++) = 35*DEGREE;
             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = 60*DEGREE;
             _target_q(_index++) = 90*DEGREE;

             _target_q(_index++) = 40*DEGREE;
             _target_q(_index++) = -75*DEGREE;
             _target_q(_index++) = -90*DEGREE;
             _target_q(_index++) = -35*DEGREE;
             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = -60*DEGREE;
             _target_q(_index++) = -90*DEGREE;

             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = -2*DEGREE;
             _target_q(_index++) = 20*DEGREE;
             _target_q(_index++) = -40*DEGREE;
             _target_q(_index++) = 20*DEGREE;
             _target_q(_index++) = 2*DEGREE;

             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = 2*DEGREE;
             _target_q(_index++) = -20*DEGREE;
             _target_q(_index++) = 40*DEGREE;
             _target_q(_index++) = -20*DEGREE;
             _target_q(_index++) = -2*DEGREE;

             _UpperCtrl.Set_Initialize();

             _UpperCtrl.SET_FK_Target(_target_q);
             _UpperCtrl.SET_FK_Parameter(1.0); // duration set

             _Joint_flag = true;
             _CLIK_flag = false;
             }
          else if (smach_state == "Valve_Ready") // Valve Ready
             {
             ROS_INFO("Valve Ready");
             _UpperCtrl.Set_Initialize();

             _target_q = q;

             _index = LA_BEGIN;
             _target_q(_index++) = 30*DEGREE;
             _target_q(_index++) = -80*DEGREE;
             _target_q(_index++) = -90*DEGREE;
             _target_q(_index++) = -100*DEGREE;
             _target_q(_index++) = 0*DEGREE;
             _target_q(_index++) = 20*DEGREE;
             _target_q(_index++) = 180*DEGREE;
             _target_q(_index++) = 0*DEGREE;

             _UpperCtrl.Set_Initialize();

             _UpperCtrl.SET_FK_Target(_target_q);
             _UpperCtrl.SET_FK_Parameter(1.0); // duration set

             _Joint_flag = true;
             _CLIK_flag = false;
             }
          else if (smach_state == "Valve_Reach") // Valve Reach
             {
             ROS_INFO("Valve Reach");
             _UpperCtrl.Set_Initialize();

             _target_x.resize(2,8);
             _target_x.setZero();
             _target_x.row(0) <<  0, 0.07, 0.1, 5*DEGREE, 0, 0, 2.0, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)
             _target_x.row(1) <<  0.1, 0, 0, 0*DEGREE, 0, 0, 2.0, 0;

             _UpperCtrl.SET_IK_Target(_target_x);
             _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

             _Joint_flag = false;
             _CLIK_flag = true;
             }
          else if (smach_state == "Valve_Close") // Valve Close
             {

             }
          else if (key_cmd == 'b') // Left Hand - 1cm up(z)
             {
             ROS_INFO("Left Hand Up");
             _UpperCtrl.Set_Initialize();

             _target_x.resize(2,8);
             _target_x.setZero();
             _target_x.row(0) <<  0, 0, 0.02, 0, 0, 0, 0.5, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)

             _UpperCtrl.SET_IK_Target(_target_x);
             _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

             _Joint_flag = false;
             _CLIK_flag = true;
             }
          else if (key_cmd == 'n') // Left Hand - 1cm down(z)
             {
             ROS_INFO("Left Hand down");
             _UpperCtrl.Set_Initialize();

             _target_x.resize(2,8);
             _target_x.setZero();
             _target_x.row(0) <<  0, 0, -0.02, 0, 0, 0, 0.5, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)

             _UpperCtrl.SET_IK_Target(_target_x);
             _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

             _Joint_flag = false;
             _CLIK_flag = true;
             }

        }
      if (_Joint_flag)
         {
                 _UpperCtrl.Set_Joint_Value(q);
                 _UpperCtrl.FK_compute(_desired_q);
         }
      else if (_CLIK_flag)
         {
                 _UpperCtrl.Set_Joint_Value(q);
                 _UpperCtrl.IK_compute(_desired_q);
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

