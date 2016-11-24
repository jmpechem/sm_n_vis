
#include "control_base.h"


const string JointName[40] = {"WaistPitch","WaistYaw",
                              "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                              "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                              "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                              "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                              "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};



int jointOccupancy[40] = {WAIST, WAIST,
                         UPPER, UPPER, UPPER, UPPER, UPPER, UPPER, UPPER,
                         UPPER, UPPER, UPPER, UPPER, UPPER, UPPER, UPPER,
                         WALKING, WALKING, WALKING, WALKING, WALKING, WALKING,
                         WALKING, WALKING, WALKING, WALKING, WALKING, WALKING,
                         HEAD, HEAD, UPPER, UPPER};

const int jointIDs[40] = {28, 27, // waist yaw - roll order
                          1,3,5,7,9,11,13,
                          2,4,6,8,10,12,14,
                          15,17,19,21,23,25,
                          16,18,20,22,24,26,
                          29,30,31,32};

// Constructor
controlBase::controlBase(ros::NodeHandle &nh) :
    uiUpdateCount(0),
    isFirstBoot(true),
    _Init_walking_flag(false),
    _Walking_flag(false),
    _Init_Egress_flag(false),
    _Egress_flag(false)

{

    total_dof = 32;

    // walkingCmdSub.initialize(3, nh, "thormang_ctrl/walking_cmd");
    walkingCmdSub = nh.subscribe("/thormang_ctrl/walking_cmd", 3, &controlBase::WalkingCmdCallback, this);
    // taskCmdSub.initialize(3, nh, "thormang_ctrl/task_cmd");
    taskCmdSub = nh.subscribe("/thormang_ctrl/task_cmd", 3, &controlBase::TaskCmdCallback, this);
    // recogCmdSub.initialize(3, nh, "thormang_ctrl/recog_cmd");
    recogCmdSub = nh.subscribe( "/thormang_ctrl/recog_cmd", 3, &controlBase::RecogCmdCallback, this);
    // jointCtrlSub.initialize(3, nh, "thormang_ctrl/joint_ctrl");
    jointCtrlSub = nh.subscribe( "/thormang_ctrl/joint_ctrl", 3, &controlBase::UIJointCtrlCallback, this);
    // smachSub.initialize(3, nh, "Jimin_machine/smach/container_status");
    smachSub = nh.subscribe( "/Jimin_machine/smach/container_status", 3, &controlBase::SmachCallback, this);
    // recogPointSub.initialize(3, nh, "custom_recog_point");
    recogPointSub = nh.subscribe( "/custom_recog_point", 3, &controlBase::RecogPointCallback, this);

    // jointStateUIPub.initialize(nh, "thormang_ctrl/joint_state",1,1,thormang_ctrl_msgs::JointState());
    jointStateUIPub.init(nh, "/thormang_ctrl/joint_state",1);
    // smachPub.initialize(nh, "transition",1,1,std_msgs::String());
    smachPub.init(nh, "/transition",1);

    // smachMsgPtr = smachPub.allocate();


    jointStateUIPub.msg_.angle.resize(total_dof);
    jointStateUIPub.msg_.velocity.resize(total_dof);
    jointStateUIPub.msg_.error.resize(total_dof);
    jointStateUIPub.msg_.current.resize(total_dof);
    jointStateUIPub.msg_.id.resize(total_dof);


    make_id_inverse_list();
    parameter_initialize();

    for(int i=0; i<total_dof; i++)
    {
        jointStateUIPub.msg_.id[i] = jointID[i];
    }
}


void controlBase::make_id_inverse_list()
{
    jointInvID.resize(50);
    for(int i=0;i<total_dof; i++)
    {
        jointID.push_back(jointIDs[i]);
        jointInvID[jointIDs[i]] = i;
    }
}



void controlBase::WalkingLoop()
{
    for (int i=0; i<28 ; i++)
        _walking_q(i) = q(i);

    if(_Init_walking_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.Init_walking_pose(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
    }
    else if (_Walking_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.compute(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
    }
    else if ( _Init_Egress_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.Egress_Init_pose(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
    }

    else if(_Egress_flag == true)
    {
        _WalkingCtrl.getdata(_walking_q,leftFootFT,rightFootFT,gyro);
        _WalkingCtrl.Egress_compute(_walking_output_q);
        // _desired_q.setZero();
        updateDesired(WALKING, _walking_output_q);
    }
    //else if (flag == true)
    {
        //_WalkingCtrl.InverseKinematics(,,,,_walking_output_q);
        //updateDesired(WALKING, _walking_output_q);
    }

}

void controlBase::WalkingCheckState()
{

    if (walkingCmdMsg.command == "init")
    {
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Init");
        _Init_walking_flag = true;
        _Walking_flag = true;
        _Egress_flag = false;
        _Init_Egress_flag = false;
        _WalkingCtrl._initialize();
    }
    else if (walkingCmdMsg.command == "start")
    {
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Start");
        _Walking_flag = true;
        _Init_walking_flag = false;
        _Egress_flag = false;
        _Init_Egress_flag = false;
        _WalkingCtrl._initialize();
        _WalkingCtrl.setApproachdata(walkingCmdMsg.x,walkingCmdMsg.y,walkingCmdMsg.theta);
       // Vector3D temp;
       // temp  << 0.0, -0.20, -0.17;
       // _WalkingCtrl.SET_IK_Target(temp);

    }
    else if (walkingCmdMsg.command == "stop")
    {
        _Walking_flag = false;
        walkingCmdMsg.command = "";
        ROS_INFO("Walking: Stop");
    }
}


void controlBase::UpperBodyLoop()
{
    //// Auto Mission /////
    if (smach_state == "Valve_Close")
    {
        if(_cnt == 0) {
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(_cnt == 5.0*_UpperCtrl._Hz){
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
        }
    }

    if(_Waist_flag){
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.WAIST_compute(_upper_output_q);
 //cout<< "waist target_q: in Upperbodyloop : "<<_target_q(0)<<"  "<<_target_q(1)<<endl;
        updateDesired(WAIST, _upper_output_q);
    }


    if (_Joint_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.FK_compute(_upper_output_q);

        //_desired_q = _upper_output_q;
        updateDesired(UPPER, _upper_output_q);
    }
    else if (_CLIK_flag)
    {
        _UpperCtrl.Set_Joint_Value(q);
        _UpperCtrl.IK_compute(_upper_output_q);
        updateDesired(UPPER, _upper_output_q);
    }

    _cnt++;
}

void controlBase::updateDesired(body_select body, VectorXD &update_q)
{
    for(int i=0; i<total_dof; i++)
    {
        if(jointOccupancy[i] == body)
        {
            _desired_q(i) = update_q(i);
        }
    }
}

void controlBase::UpperBodyCheckState()
{

    if(check_state_changed())
    {
        if (smach_state == "Handclap_Ready")
        {
            ROS_INFO("Handclap_Ready");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = 0.000863797;
            _target_q(_index++) = 1.77324;
            _target_q(_index++) = 1.06179;
            _target_q(_index++) = 1.82509;
            _target_q(_index++) =  -0.0544568;
            _target_q(_index++) = -0.561702;
            _target_q(_index++) = 1.58122;

         // L_arm
            _target_q(_index++) =  40*DEGREE;
            _target_q(_index++) = -95*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -70*DEGREE;
            _target_q(_index++) = -10*DEGREE;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;

            _target_q(RA_HAND) = 40*DEGREE;
            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Waist_flag = true;
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(smach_state == "Handclap_Do")
        {
            ROS_INFO("Handclap_Do");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = -0.0517114;
            _target_q(_index++) = 1.80345;
            _target_q(_index++) = 1.26196;
            _target_q(_index++) = 1.76603;
            _target_q(_index++) =  -0.000976466;
            _target_q(_index++) = 0.410393;
            _target_q(_index++) = 1.63905;

         // L_arm
            _target_q(_index++) =  40*DEGREE;
            _target_q(_index++) = -95*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -70*DEGREE;
            _target_q(_index++) = -10*DEGREE;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;

             _UpperCtrl.Set_Initialize();

            double loop_time = 3.0*_UpperCtrl._Hz;

            if(_cnt < loop_time)
            {
                 _UpperCtrl.Set_Initialize();
                _target_q(5) = _target_q(5) +10*DEGREE;
                _UpperCtrl.SET_FK_Target(_target_q);
                _UpperCtrl.SET_FK_Parameter(loop_time); // duration set
            }
            else if(_cnt >= loop_time && _cnt < loop_time*2)
            {


                 _UpperCtrl.SET_FK_Target(_target_q);
                 _UpperCtrl.SET_FK_Parameter(loop_time); // duration set

            }


            _Waist_flag = true;
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(smach_state == "Handclap_End")
        {
        }
        else if(smach_state == "Handshake_Ready")
        {
            ROS_INFO("Handshake_Ready");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = 0.300263;
            _target_q(_index++) = 1.79158;
            _target_q(_index++) = 1.31043;
            _target_q(_index++) = 1.69177;
            _target_q(_index++) =  0.328656;
            _target_q(_index++) = 1.36793;
            _target_q(_index++) = 1.12177+90*DEGREE;

         // L_arm
            _target_q(_index++) = -0.300263;
            _target_q(_index++) = -1.79158;
            _target_q(_index++) = -1.31043;
            _target_q(_index++) = -1.69177;
            _target_q(_index++) =  -0.328656;
            _target_q(_index++) = -1.36793;
            _target_q(_index++) = -1.12177;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(2.0); // duration set

            _Waist_flag = true;
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if(smach_state == "Handshake_Do")
        {
        }
        else if(smach_state == "Handshake_End")
        {
        }
        else if (smach_state == "Valve_Init") // Valve Init
        {
            ROS_INFO("Valve Init");

            _index = 0;
            _target_q(_index++) = 0;
            _target_q(_index++) = 0;

          // R_arm
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 95*DEGREE;
            _target_q(_index++) = 80*DEGREE;
            _target_q(_index++) = 110*DEGREE;//70*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = 70*DEGREE;
            _target_q(_index++) = 100*DEGREE;

         // L_arm
            _target_q(_index++) =  40*DEGREE;
            _target_q(_index++) = -95*DEGREE;
            _target_q(_index++) = -80*DEGREE;
            _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -70*DEGREE;
            _target_q(_index++) = -10*DEGREE;
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Waist_flag = true;
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Ready") // Valve Ready
        {
             ROS_INFO("Valve Ready");

            _index = 2;
           /* _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 100*DEGREE;
            _target_q(_index++) = 90*DEGREE;
            _target_q(_index++) = -45*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 90*DEGREE;
           // _target_q(_index++) = 0*DEGREE;

            _target_q(_index++) =  0*DEGREE;
            _target_q(_index++) = -100*DEGREE;
            _target_q(_index++) = -90*DEGREE;
            _target_q(_index++) = 45*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -90*DEGREE;*/

 	 
  /*        // R_arm
            _target_q(_index++) =  -40*DEGREE;
            _target_q(_index++) =   95*DEGREE;
            _target_q(_index++) =   80*DEGREE;
            _target_q(_index++) =   70*DEGREE;
            _target_q(_index++) =    0*DEGREE;
            _target_q(_index++) =   70*DEGREE;
            _target_q(_index++) =   10*DEGREE;

         // L_arm
            _target_q(_index++) =   75*DEGREE;
            _target_q(_index++) =  -85*DEGREE;
            _target_q(_index++) =  -80*DEGREE;
            _target_q(_index++) = -110*DEGREE;
            _target_q(_index++) =   -5*DEGREE;
            _target_q(_index++) =  -60*DEGREE;
            _target_q(_index++) =  -15*DEGREE;
*/

            // R_arm for annae
              _target_q(_index++) =  0.0*DEGREE;
              _target_q(_index++) =   95*DEGREE;
              _target_q(_index++) =   50*DEGREE;
              _target_q(_index++) =   110*DEGREE;
              _target_q(_index++) =   -60*DEGREE;
              _target_q(_index++) =   45*DEGREE;
              _target_q(_index++) =   10*DEGREE;

           // L_arm
              _target_q(_index++) =   0.0*DEGREE;
              _target_q(_index++) =  -105*DEGREE;
              _target_q(_index++) =  -110*DEGREE;
              _target_q(_index++) = -90*DEGREE;
              _target_q(_index++) =   -0*DEGREE;
              _target_q(_index++) =  -20*DEGREE;
              _target_q(_index++) =  -10*DEGREE;


            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(2.0); // duration set
     
            _Waist_flag = true;
            _Joint_flag = true;
            _CLIK_flag = false;
        }
        else if (smach_state == "Valve_Reach") // Valve Reach
        {
            ROS_INFO("Valve Reach");

            _target_x.resize(2,8);
            _target_x.setZero();

            Vector6D    vision_data;
            vision_data.setZero();
            /*
	    vision_data(0) = recogPoint[0]; // 48.18*0.01;//+0.04;  M
            vision_data(1) = recogPoint[1]; // 17.62*0.01;//+0.02;
            vision_data(2) = recogPoint[2]; // 13.25*0.01;//+0.24;
            ROS_INFO("%f, %f, %f",recogPoint[0],recogPoint[1],recogPoint[2]);
            _target_x.row(0) << -1, vision_data(1), vision_data(2), vision_data(3), vision_data(4), vision_data(5),  5.0, 0;
            _target_x.row(1) << vision_data(0), vision_data(1), vision_data(2), 0.0, 0.0, 0.0, 5.0, 0;*/

            vision_data(0) = 0.15; // 48.18*0.01;//+0.04;  M
            vision_data(1) = -0.01; // 17.62*0.01;//+0.02;
            vision_data(2) = 0.0; // 13.25*0.01;//+0.24;
            _target_x.row(0) << 0, vision_data(1), vision_data(2), vision_data(3), vision_data(4), vision_data(5),  5.0, 0;
            _target_x.row(1) << vision_data(0), vision_data(1), vision_data(2), 0.0, 0.0, 0.0, 5.0, 0;

            //   _target_x.row(0) <<  0, 0.07, 0.1, 5*DEGREE, 0, 0, 1.0, 0; // x,y,z,a,b,r,duration, Right(1) or Left(0)
            //  _target_x.row(1) <<  0.1, 0, 0, 0*DEGREE, 0, 0, 1.0, 0;
/*
            if(vision_data(0) > 0.67 || vision_data(1) > 0.35 || vision_data(1) < 0.12 || vision_data(2) > 0.225 || vision_data(2) < -0.403 ){
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
                //_target_q(_index++) = 0*DEGREE;

                _UpperCtrl.SET_FK_Target(_target_q);
                _UpperCtrl.SET_FK_Parameter(5.0); // duration set

                _Joint_flag = true;
                _CLIK_flag = false;

            }
            else{*/
                _UpperCtrl.Set_Initialize();
                _UpperCtrl.SET_IK_Target(_target_x);
                _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

                _Waist_flag = false;
                _Joint_flag = false;
                _CLIK_flag = true;
            
        }
        else if (smach_state == "Valve_Close") // Valve Close
        {
            _target_q=q;
            _target_q(LA_BEGIN+6) = _target_q(LA_BEGIN+6) - 390*DEGREE;
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0;

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
        }

        // Door missions
        else if (smach_state == "Door_Init") // Door Init
        {
            ROS_INFO("Door Init");

             _target_q = q;

            _index = 0;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0*DEGREE;

            // R_arm
              _target_q(_index++) = -10*DEGREE;
              _target_q(_index++) = 110*DEGREE;
              _target_q(_index++) = 80*DEGREE;
              _target_q(_index++) = 150*DEGREE;//70*DEGREE;
              _target_q(_index++) =  0*DEGREE;
              _target_q(_index++) = 45*DEGREE;
              _target_q(_index++) = 10*DEGREE;
           // L_arm
              _target_q(_index++) =  0*DEGREE;
              _target_q(_index++) = -80*DEGREE;
              _target_q(_index++) = -90*DEGREE;
              _target_q(_index++) = -160*DEGREE;//-70*DEGREE;
              _target_q(_index++) =  -0*DEGREE;
              _target_q(_index++) = 10*DEGREE;
              _target_q(_index++) = 0*DEGREE;
            /* _target_q(_index++) =  29*DEGREE;
             _target_q(_index++) = -85*DEGREE;
             _target_q(_index++) = -56*DEGREE;
             _target_q(_index++) = -140*DEGREE;
             _target_q(_index++) =  68*DEGREE;
             _target_q(_index++) = 32*DEGREE;
             _target_q(_index++) = 10*DEGREE;*/
         //R_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = -2*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = -40*DEGREE;
            _target_q(_index++) = 20*DEGREE;
            _target_q(_index++) = 2*DEGREE;
        //L_leg
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 2*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = 40*DEGREE;
            _target_q(_index++) = -20*DEGREE;
            _target_q(_index++) = -2*DEGREE;


            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;
        }

        else if (smach_state == "Door_Ready") // Door Ready
        {
            ROS_INFO("Door Ready");

               _target_q = q;

                ROS_INFO("1st motion");

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(5.0); // duration set

            _Joint_flag = true;
            _CLIK_flag = false;

        }
    }

    if (taskCmdMsg.subtask==1)
    {
        ROS_INFO("Task Controller for arms");
        _UpperCtrl.Set_Initialize();

        _target_x.resize(1,8);
        _target_x.setZero();
        double run_time = 0.0f;
        run_time = 5.0;
        std::cout << taskCmdMsg.x << taskCmdMsg.y << taskCmdMsg.z << taskCmdMsg.roll << taskCmdMsg.pitch << taskCmdMsg.yaw << run_time << taskCmdMsg.arm << std::endl;
        _target_x.row(0) <<  taskCmdMsg.x*0.01, taskCmdMsg.y*0.01, taskCmdMsg.z*0.01, taskCmdMsg.roll*DEGREE, taskCmdMsg.pitch*DEGREE, taskCmdMsg.yaw*DEGREE, run_time, taskCmdMsg.arm; // x,y,z,a,b,r,duration, Right(1) or Left(0)

        _UpperCtrl.SET_IK_Target(_target_x);
        _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
        _Joint_flag = false;
        _CLIK_flag = true;
        _Waist_flag = false;
        taskCmdMsg.subtask=taskCmdMsg.NONE;
    }

    if(jointCtrlMsgRecv == true)
    {
        _UpperCtrl.Set_Initialize();
        jointCtrlMsgRecv = false;
        double nowPos = q(jointInvID[jointCtrlMsg.id]);
        double aimPos = nowPos + jointCtrlMsg.angle * DEGREE;
        double run_time = fabs(jointCtrlMsg.angle)/10.0 ;

        _target_q = q;
        _target_q(jointInvID[jointCtrlMsg.id]) = aimPos;


        _UpperCtrl.SET_FK_Target(_target_q);
        _UpperCtrl.SET_FK_Parameter(run_time); // duration set

        _Joint_flag = true;
        _Waist_flag = true;
        _CLIK_flag = false;
    }
}

void controlBase::WholebodyLoop(){

    //////////
    int sequence = 16;
    VectorXD gap_egress;
    gap_egress.resize(sequence);
    gap_egress.setZero();
    VectorXD t_egress;
    t_egress.resize(sequence+1);
    t_egress(0) = 0;
    for (int i=0; i<sequence; i++)
    {
        gap_egress(i) = _UpperCtrl._gap*3/4;
        gap_egress(0) = _UpperCtrl._gap2;
        gap_egress(2) = _UpperCtrl._gap/2; // dari chum olim
        gap_egress(5) = _UpperCtrl._gap; // moum deum
        //gap_egress(6) = _UpperCtrl._gap;
        gap_egress(7) = _UpperCtrl._gap/2;
        //gap_egress(8) = _UpperCtrl._gap/2;
        gap_egress(9) = _UpperCtrl._gap;

        /*
        gap_egress(i) = gap*3/4;
        gap_egress(0) = gap2;
        gap_egress(2) = gap/2; // dari chum olim
        gap_egress(5) = gap; // moum deum
        gap_egress(6) = gap;
        //gap_egress(7) = gap/2; */


        t_egress(i+1) = t_egress(i)+gap_egress(i);
    }
    ////////////


    if(smach_state == "Handshake_Do")
    {
        _Waist_flag = true;
        _Joint_flag = true;
        _CLIK_flag = false;



       double loop_time1 = 1.0*200.0;



       if(_cnt == 0)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

           // R_arm
             _target_q(_index++) = 0.190862;
             _target_q(_index++) = 1.80709;
             _target_q(_index++) = 1.3119;
             _target_q(_index++) = 1.69628;
             _target_q(_index++) =  -0.200664;
             _target_q(_index++) = 1.49353;
             _target_q(_index++) = 1.46811+90*DEGREE;

          // L_arm
             _target_q(_index++) = -0.190862;
             _target_q(_index++) = -1.80709;
             _target_q(_index++) = -1.3119;
             _target_q(_index++) = -1.69628;
             _target_q(_index++) =  0.200664;
             _target_q(_index++) = -1.49353;
             _target_q(_index++) = -1.46811;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();
           _UpperCtrl.SET_FK_Target(_target_q);
           _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }
       else if(_cnt == loop_time1)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

           // R_arm
             _target_q(_index++) = 0.300263;
             _target_q(_index++) = 1.79158;
             _target_q(_index++) = 1.31043;
             _target_q(_index++) = 1.69177;
             _target_q(_index++) =  0.328656;
             _target_q(_index++) = 1.36793;
             _target_q(_index++) = 1.12177+90*DEGREE;

          // L_arm
             _target_q(_index++) = -0.300263;
             _target_q(_index++) = -1.79158;
             _target_q(_index++) = -1.31043;
             _target_q(_index++) = -1.69177;
             _target_q(_index++) =  -0.328656;
             _target_q(_index++) = -1.36793;
             _target_q(_index++) = -1.12177;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

             _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }
       else if(_cnt ==loop_time1*2.0)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

           // R_arm
             _target_q(_index++) = 0.190862;
             _target_q(_index++) = 1.80709;
             _target_q(_index++) = 1.3119;
             _target_q(_index++) = 1.69628;
             _target_q(_index++) =  -0.200664;
             _target_q(_index++) = 1.49353;
             _target_q(_index++) = 1.46811+90*DEGREE;

          // L_arm
             _target_q(_index++) = -0.190862;
             _target_q(_index++) = -1.80709;
             _target_q(_index++) = -1.3119;
             _target_q(_index++) = -1.69628;
             _target_q(_index++) =  0.200664;
             _target_q(_index++) = -1.49353;
             _target_q(_index++) = -1.46811;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();
           _UpperCtrl.SET_FK_Target(_target_q);
           _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }
       else if(_cnt == loop_time1*3.0)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

           // R_arm
             _target_q(_index++) = 0.300263;
             _target_q(_index++) = 1.79158;
             _target_q(_index++) = 1.31043;
             _target_q(_index++) = 1.69177;
             _target_q(_index++) =  0.328656;
             _target_q(_index++) = 1.36793;
             _target_q(_index++) = 1.12177+90*DEGREE;

          // L_arm
             _target_q(_index++) = -0.300263;
             _target_q(_index++) = -1.79158;
             _target_q(_index++) = -1.31043;
             _target_q(_index++) = -1.69177;
             _target_q(_index++) =  -0.328656;
             _target_q(_index++) = -1.36793;
             _target_q(_index++) = -1.12177;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

             _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }

    }

    else if (smach_state == "Handclap_Do")
    {
       // ROS_INFO("Handclap_Do");

        _Waist_flag = true;
        _Joint_flag = true;
        _CLIK_flag = false;

       double loop_time1 = 0.5*200.0;


       if(_cnt == 0)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

         // R_arm
           _target_q(_index++) = 0.0822986;
           _target_q(_index++) = 1.77326;
           _target_q(_index++) = 1.07096;
           _target_q(_index++) = 2.05003;
           _target_q(_index++) =  -0.0545319;
           _target_q(_index++) = -0.511321;
           _target_q(_index++) = 1.58081;

        // L_arm
           _target_q(_index++) =  40*DEGREE;
           _target_q(_index++) = -95*DEGREE;
           _target_q(_index++) = -80*DEGREE;
           _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
           _target_q(_index++) =  0*DEGREE;
           _target_q(_index++) = -70*DEGREE;
           _target_q(_index++) = -10*DEGREE;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

            _UpperCtrl.Set_Initialize();
           _UpperCtrl.SET_FK_Target(_target_q);
           _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }
       else if(_cnt == loop_time1)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

         // R_arm
           _target_q(_index++) = 0.0601153;
           _target_q(_index++) = 1.77348;
           _target_q(_index++) = 1.07093;
           _target_q(_index++) = 1.69472;
           _target_q(_index++) =  -0.0544818;
           _target_q(_index++) = -0.511321;
           _target_q(_index++) = 1.58079;

        // L_arm
           _target_q(_index++) =  40*DEGREE;
           _target_q(_index++) = -95*DEGREE;
           _target_q(_index++) = -80*DEGREE;
           _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
           _target_q(_index++) =  0*DEGREE;
           _target_q(_index++) = -70*DEGREE;
           _target_q(_index++) = -10*DEGREE;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

             _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(loop_time1/200); // duration set
       }
       else if(_cnt ==loop_time1*2.0)
       {
           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

           // R_arm
             _target_q(_index++) = 0.0822986;
             _target_q(_index++) = 1.77326;
             _target_q(_index++) = 1.07096;
             _target_q(_index++) = 2.05003;
             _target_q(_index++) =  -0.0545319;
             _target_q(_index++) = -0.511321;
             _target_q(_index++) = 1.58081;
        // L_arm
           _target_q(_index++) =  40*DEGREE;
           _target_q(_index++) = -95*DEGREE;
           _target_q(_index++) = -80*DEGREE;
           _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
           _target_q(_index++) =  0*DEGREE;
           _target_q(_index++) = -70*DEGREE;
           _target_q(_index++) = -10*DEGREE;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

           _UpperCtrl.Set_Initialize();
          _UpperCtrl.SET_FK_Target(_target_q);
          _UpperCtrl.SET_FK_Parameter(loop_time1/200);
       }
       else if(_cnt == loop_time1*3.0)
       {

           _index = 0;
           _target_q(_index++) = 0;
           _target_q(_index++) = 0;

         // R_arm
           _target_q(_index++) = 0.000863797;
           _target_q(_index++) = 1.77324;
           _target_q(_index++) = 1.06179;
           _target_q(_index++) = 1.82509;
           _target_q(_index++) =  -0.0544568;
           _target_q(_index++) = -0.561702;
           _target_q(_index++) = 1.58122;

        // L_arm
           _target_q(_index++) =  40*DEGREE;
           _target_q(_index++) = -95*DEGREE;
           _target_q(_index++) = -80*DEGREE;
           _target_q(_index++) = -110*DEGREE;//-70*DEGREE;
           _target_q(_index++) =  0*DEGREE;
           _target_q(_index++) = -70*DEGREE;
           _target_q(_index++) = -10*DEGREE;
        //R_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = -2*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = -40*DEGREE;
           _target_q(_index++) = 20*DEGREE;
           _target_q(_index++) = 2*DEGREE;
       //L_leg
           _target_q(_index++) = 0*DEGREE;
           _target_q(_index++) = 2*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = 40*DEGREE;
           _target_q(_index++) = -20*DEGREE;
           _target_q(_index++) = -2*DEGREE;

           _UpperCtrl.Set_Initialize();
          _UpperCtrl.SET_FK_Target(_target_q);
          _UpperCtrl.SET_FK_Parameter(loop_time1/200);
       }

    }

    else if (smach_state == "Door_Init"){
        if(_cnt == 0){
            _Joint_flag = true;
            _CLIK_flag = false;

            _Waist_flag = true; // for waist only - daum tic eseo off hayaham

            _Init_walking_flag = false;
            _Walking_flag = false;
            _Init_Egress_flag = true;
            _Egress_flag = false;
            _WalkingCtrl._initialize();


        }
        else if(_cnt == t_egress(1)){

            _index = LA_BEGIN;
            // L_arm
               _target_q(_index++) =  0*DEGREE;
               _target_q(_index++) = -30*DEGREE;
               _target_q(_index++) = -90*DEGREE;
               _target_q(_index++) = -160*DEGREE;//-70*DEGREE;
               _target_q(_index++) =  -0*DEGREE;
               _target_q(_index++) = 10*DEGREE;
               _target_q(_index++) = 0*DEGREE;

               _UpperCtrl.Set_Initialize();

               _UpperCtrl.SET_FK_Target(_target_q);
               _UpperCtrl.SET_FK_Parameter(gap_egress(1)/_UpperCtrl._Hz); // duration set

               _Joint_flag = true;
               _CLIK_flag = false;

            _UpperCtrl._cnt = 0;
            _Waist_flag = false;

            printf("1");
        }/*
        else if(_cnt == _UpperCtrl._gap + _UpperCtrl._gap2){
            //cout<<"loop 4"<<_cnt<<endl;
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _UpperCtrl._gap, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            printf("2");

        }*/

    }
    else if(smach_state=="Door_Ready"){
        if(_cnt==0){
        _Walking_flag = false;
        _Init_walking_flag = false;
        _Init_Egress_flag = false;
        _Egress_flag = true;
        _WalkingCtrl._initialize();
        }
      else if(_cnt == t_egress(3)){
            // for waist start
            _index = 0;
            _target_q(_index++) = 35*DEGREE;
            _target_q(_index++) = 0;

            // for waist end

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.105*sin(35*DEGREE), -0.105*(1-cos(35*DEGREE)), 0.0, 0.0, 0.0, 0*DEGREE, gap_egress(3)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            // for waist start//
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(gap_egress(3)/_UpperCtrl._Hz); // duration set
            // for waist edn//
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Waist_flag = true; // for waist only - daum tic eseo off hayaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("3");

        }
        else if(_cnt == t_egress(4)){

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(4)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Waist_flag=false; // ap eso of 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("4");
        }
        else if(_cnt == t_egress(5)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.25*cos(35*DEGREE)+0.15*sin(35*DEGREE), 0.25*sin(35*DEGREE)+0.15*cos(35*DEGREE), -0.1, 0.0, 0.0, 0.0, gap_egress(5)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("5");
        }
        else if(_cnt == t_egress(6)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(6)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold


            cout<<" gyro data in 6 sequence-- roll: "<<gyro[0]<<", pitch :  "<<gyro[1]<<endl;

            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("6");
        }
        else if(_cnt == t_egress(7)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(7)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            cout<<" gyro data in 7 sequence-- roll: "<<gyro[0]<<", pitch :  "<<gyro[1]<<endl;

            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("7");
        }
        else if(_cnt == t_egress(8)){
            // for waist start
            _index = 0;
            _target_q(_index++) = 70*DEGREE;
            _target_q(_index++) = 0;

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.105*(sin(70*DEGREE)-sin(35*DEGREE))+0.02052*sin(70*DEGREE), 0.105*(-cos(70*DEGREE)+cos(35*DEGREE))+0.02052*cos(70*DEGREE), 0.0, 0.0, 0.0, 0.0, gap_egress(8)/_UpperCtrl._Hz, 0;
            //-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)) = 0.02052

            _UpperCtrl.Set_Initialize();
            // for waist start//
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(gap_egress(8)/_UpperCtrl._Hz); // duration set
            // for waist edn//
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            cout<<" gyro data in 8 sequence-- roll: "<<gyro[0]<<", pitch :  "<<gyro[1]<<endl;

            _Waist_flag = true; // for waist only - daum tic eseo off hayaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("8");
        }
        else if(_cnt == t_egress(9)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.06917*cos(70*DEGREE), 0.06917*sin(70*DEGREE), 0.0, 0.0, 0.0, 0.0, gap_egress(9)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
            //constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)) = -0.06917

            _Waist_flag=false; // ap eso on 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("9");
        }
        else if(_cnt == t_egress(10)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.2112/2*cos(70*DEGREE), 0.2112/2*sin(70*DEGREE), 0.1, 0.0, 0.0, 0.0, gap_egress(10)/_UpperCtrl._Hz, 0;

            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Waist_flag=false; // ap eso on 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("10");
        }
        else if(_cnt == t_egress(11)){
            _target_q = q;
            _target_q(LA_HAND) = 0.0*DEGREE;

            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(2.0); // duration set //constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)) = -0.2112

            _Waist_flag=false; // ap eso of 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = true;
            _CLIK_flag = false;
            //printf("11");
        }
        else if(_cnt == t_egress(12)){ // un grip hand
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(12)/_UpperCtrl._Hz, 0;
            //�� ����


            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
            //constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)) = -0.2112

            _Waist_flag=false; // ap eso of 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("12");
        }
        else if(_cnt == t_egress(13)){
            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.25, -0.15, 0.0, 0.0, 0.0, gap_egress(13)/_UpperCtrl._Hz, 0;
            //�� ������


            _UpperCtrl.Set_Initialize();
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, false, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold
            //constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)) = -0.2112

            _Waist_flag=false; // ap eso of 2myeon, yegiso off heyaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("13");
        }
        else if(_cnt == t_egress(14)){
            // for waist start
            _index = 0;
            _target_q(_index++) = 0*DEGREE;
            _target_q(_index++) = 0;

            _target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(14)/_UpperCtrl._Hz, 0;


            _UpperCtrl.Set_Initialize();
            // for waist start//
            _UpperCtrl.SET_FK_Target(_target_q);
            _UpperCtrl.SET_FK_Parameter(gap_egress(14)/_UpperCtrl._Hz); // duration set
            // for waist edn//
            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Waist_flag = true; // for waist only - daum tic eseo off hayaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;
            //printf("14");
        }
        else if(_cnt == t_egress(15)){
              _index = LA_BEGIN;
              // L_arm
              _target_q(_index++) =  -30*DEGREE;
              _target_q(_index++) = -120*DEGREE;
              _target_q(_index++) = -120*DEGREE;
              _target_q(_index++) = -75*DEGREE;//-70*DEGREE;
              _target_q(_index++) =  -0*DEGREE;
              _target_q(_index++) = -15*DEGREE;
              _target_q(_index++) = 90*DEGREE;
              _index = RA_BEGIN;
              // R_arm
              _target_q(_index++) =  0*DEGREE;
              _target_q(_index++) = 105*DEGREE;
              _target_q(_index++) = 55*DEGREE;
              _target_q(_index++) = 105*DEGREE;//-70*DEGREE;
              _target_q(_index++) =  -0*DEGREE;
              _target_q(_index++) = 15*DEGREE;
              _target_q(_index++) = 90*DEGREE;


               _UpperCtrl.Set_Initialize();

               _UpperCtrl.SET_FK_Target(_target_q);
               _UpperCtrl.SET_FK_Parameter(gap_egress(15)/_UpperCtrl._Hz); // duration set

               _Joint_flag = true;
               _CLIK_flag = false;

            _UpperCtrl._cnt = 0;
            _Waist_flag = false;


            /*_target_x.resize(1,8);
            _target_x.setZero();
            _target_x.row(0) <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gap_egress(15)/_UpperCtrl._Hz, 0;


            _UpperCtrl.Set_Initialize();

            _UpperCtrl.SET_IK_Target(_target_x);
            _UpperCtrl.SET_IK_Parameter(100.0, true, true, 0.05, 0.001); // CLIK gain, Rel of Abs Pos, Singularity Avoidance, Singularity Gain, Singularity Threshold

            _Waist_flag = false; // for waist only - daum tic eseo off hayaham
            _UpperCtrl._cnt = 0;
            _Joint_flag = false;
            _CLIK_flag = true;*/
            //printf("15");
        }
    }
//ROS_INFO;

}

void controlBase::update()
{

}

void controlBase::compute()
{
    // Update
    WalkingCheckState();
    UpperBodyCheckState();


    // Planning


    // Loop
    //if(wholeflag==true)
   // {
        // Loop();
   // }
   // else
   // {
        WholebodyLoop();
        UpperBodyLoop();
        WalkingLoop();
    //}

  /*  if(smach_state == "Valve_Reach" ||smach_state == "Valve_Ready" || smach_state == "Valve_Close")
    {
int suhan;
		suhan=RA_BEGIN;
	    _desired_q(suhan++) = -40*DEGREE;
            _desired_q(suhan++) = 75*DEGREE;
            _desired_q(suhan++) = 90*DEGREE;
            _desired_q(suhan++) = 35*DEGREE;
            _desired_q(suhan++) = 0*DEGREE;
            _desired_q(suhan++) = 60*DEGREE;
            _desired_q(suhan++) = 90*DEGREE;
    }*/
}

void controlBase::reflect()
{

    if(++uiUpdateCount > 4)
    {
        uiUpdateCount = 0;
        for(int i=0; i<total_dof; i++)
        {
            jointStateUIPub.msg_.angle[i] = q(i);
            jointStateUIPub.msg_.velocity[i] = q_dot(i);
            jointStateUIPub.msg_.current[i] = torque(i);
        }

        if (jointStateUIPub.trylock()) {
            jointStateUIPub.unlockAndPublish();
        }
    }
}
// Common functions
bool controlBase::check_state_changed()
{
    if(before_state != smach_state)
    {
        before_state = smach_state;
        _UpperCtrl.Set_Initialize();
        _cnt = 0;
        _Joint_flag = false;
        _CLIK_flag = false;
        return true;
    }
    return false;
}

void controlBase::parameter_initialize()
{
    q.resize(total_dof); q.setZero();
    q_dot.resize(total_dof); q_dot.setZero();
    torque.resize(total_dof); torque.setZero();
    leftFootFT.setZero();  rightFootFT.setZero(); gyro.setZero();
    _desired_q.resize(total_dof); _desired_q.setZero();
    _target_q.resize(total_dof); _target_q.setZero();
    _upper_output_q.resize(total_dof); _upper_output_q.setZero();
    _walking_output_q.resize(28); _walking_output_q.setZero();
    _walking_q.resize(28); _walking_q.setZero();
}
void controlBase::readdevice()
{
    ros::spinOnce();
}

int controlBase::getch()
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
        if(read(filedesc, &buff, len )) {} // just for unused

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
double controlBase::Rounding( double x, int digit )
{
    return ( floor( (x) * pow( float(10), digit ) + 0.5f ) / pow( float(10), digit ) );
}

void controlBase::WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg)
{
    walkingCmdMsg = *msg;
}

void controlBase::TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg)
{
  taskCmdMsg = *msg;
}

void controlBase::RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg)
{

}

void controlBase::UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr &joint)
{
    jointCtrlMsg = *joint;
    jointCtrlMsgRecv = true;
}

void controlBase::SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach)
{
    smach_state = smach->active_states[0];
}

void controlBase::RecogPointCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    for(int i=0; i<6; i++)
    {
        recogPoint[i] = msg->data[i];
    }
}

/*
// Callback
void controlBase::UIJointCtrlCallback(const thormang_ctrl_msgs::JointSetConstPtr &joint)
{
    jointCtrlMsg = *joint;
    jointCtrlMsgRecv = true;
}

void controlBase::SmachCallback(const smach_msgs::SmachContainerStatusConstPtr &smach)
{
    smach_state = smach->active_states[0];
}

void controlBase::WalkingCmdCallback(const thormang_ctrl_msgs::WalkingCmdConstPtr& msg)
{
    walkingCmdMsg = *msg;
}

void controlBase::TaskCmdCallback(const thormang_ctrl_msgs::TaskCmdConstPtr& msg)
{
  taskCmdMsg = *msg;
}

void controlBase::RecogCmdCallback(const thormang_ctrl_msgs::RecogCmdConstPtr& msg)
{

}
*/
