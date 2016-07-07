#include "Walking_Controller.h"

WalkingCtrl::WalkingCtrl()
{

    for(int i=0; i<FILE_CNT;i++)
    {
        file[i].open(FILE_NAMES[i].c_str(),ios_base::out);
    }

}
WalkingCtrl::~WalkingCtrl()
{

    for(int i=0; i<FILE_CNT;i++)
    {
        if(file[i].is_open())
            file[i].close();
    }

}

void WalkingCtrl::Init_walking_pose(VectorXD& output)
{
    if(_cnt == 0)
    {
        _init_q = _q;
    }
    VectorXD target_q;
    target_q.resize(28);
    target_q.setZero();
    int index = 0;

    /*target_q(index++) = 90*DEGREE;
    target_q(index++) = 0;


    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = 0;

    target_q(index++) = 0;
    target_q(index++) = 0;

    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = 0;

    target_q(index++) = 0;
    target_q(index++) = 0;

    target_q(index++) = 0;*/

    target_q(index++) = 0;
    target_q(index++) = 0;
    target_q(index++) = -40*DEGREE;
    target_q(index++) = 75*DEGREE;
    target_q(index++) = 90*DEGREE;

    target_q(index++) = 35*DEGREE;
    target_q(index++) = 0*DEGREE;
    target_q(index++) = 60*DEGREE;
    target_q(index++) = 90*DEGREE;

    target_q(index++) = 40*DEGREE;
    target_q(index++) = -75*DEGREE;
    target_q(index++) = -90*DEGREE;
    target_q(index++) = -35*DEGREE;
    target_q(index++) = 0*DEGREE;
    target_q(index++) = -60*DEGREE;
    target_q(index++) = -90*DEGREE;

    target_q(index++) = 0*DEGREE;
    target_q(index++) = -2*DEGREE;
    target_q(index++) = 25*DEGREE;
    target_q(index++) = -50*DEGREE;
    target_q(index++) = 25*DEGREE;
    target_q(index++) = 2*DEGREE;

    target_q(index++) = 0*DEGREE;
    target_q(index++) = 2*DEGREE;
    target_q(index++) = -25*DEGREE;
    target_q(index++) = 50*DEGREE;
    target_q(index++) = -25*DEGREE;
    target_q(index++) = -2*DEGREE;
/*
    target_q.setZero();
    target_q(1) = 90*DEGREE;*/
    for (int i=0; i<index; i++)
        output(i) = Cubic(_cnt,0.0,1.0*Hz,_init_q(i),0.0,target_q(i),0.0);

    _cnt++;

}



void WalkingCtrl::compute(VectorXD& output)
{
    if(_cnt < _T_Last + 10*Hz)
    {
        if(_cnt == 0)
        {

            cout << "scan_data" << _scan_data << endl;
            plan_foot_step(_scan_data,_foot_step,_Step_Planning_flag);
            cout << "foot_Step" << _foot_step << endl;
        }

        Step_count(_foot_step);

        if(_cnt == _T_Start)
         ROS_INFO("%d %d",_cnt,_step_number);


        if(_cnt>_T_Start)
        {
            _q=_desired_q_notcompensate;
        }

        Robot_state_update();

        //if(_cnt>_T_Start)
        //{
            //outputHipcompensation();
        //}

        Vector3D Lfoot = _T_LFoot_support[5].translation();
        Vector3D Rfoot = _T_RFoot_support[5].translation();
        ZMP_real(_L_FT_global, _R_FT_global, Lfoot, Rfoot, _ZMP_real);


        if(_cnt == _T_Start && _step_number != 0)
        {
            _COM_update_flag = true;
        }
        _init_state_update();

        if(_cnt == 0)
        {
           // ROS_INFO("%d %d",_cnt,_step_number);
            Ref_ZMP_update();
        }
        else if(_cnt == _T_Start && _step_number != 0)
        {
          // ROS_INFO("%d %d",_cnt,_step_number);
            Ref_ZMP_update();
        }

        Ref_COM_update_local();

        Trunk_trajectory_update();

        Foot_trajectory_update();

        Change_Global_Pattern();

      // Impedance_controller();

        //Change_Local_Pattern();

        //double k = 100.0;
        //for (int i=0; i<3; i++)
        //{
        //	if(_foot_step(_step_number,6) == 1) //�޹�����
        //	{
        //		Desired_RFoot_dot(i) = k*(Foot_trajectory.RFoot.translation()(i)-_T_RFoot_support[5].translation()(i));
        //		Desired_RFoot_dot(i+3) = k*(Foot_trajectory.RFoot_euler(i)-_T_RFoot_support_euler(i));
        //		//Desired_LFoot_dot.setZero();
        //		Desired_LFoot_dot(i) = k*(Foot_trajectory.LFoot.translation()(i)-_T_LFoot_support[5].translation()(i));
        //		Desired_LFoot_dot(i+3) = k*(Foot_trajectory.LFoot_euler(i)-_T_LFoot_support_euler(i));

        //	}
        //	else
        //	{
        //		Desired_LFoot_dot(i) = k*(Foot_trajectory.LFoot.translation()(i)-_T_LFoot_support[5].translation()(i));
        //		Desired_LFoot_dot(i+3) = k*(Foot_trajectory.LFoot_euler(i)-_T_LFoot_support_euler(i));
        //		//Desired_RFoot_dot.setZero();
        //		Desired_RFoot_dot(i) = k*(Foot_trajectory.RFoot.translation()(i)-_T_RFoot_support[5].translation()(i));
        //		Desired_RFoot_dot(i+3) = k*(Foot_trajectory.RFoot_euler(i)-_T_RFoot_support_euler(i));
        //	}

        //}


        Vector3D trunk_temp;
        Rot2euler(Trunk_trajectory.linear(),trunk_temp);
        if(_cnt>_T_Start)
        {
            file[2] << _cnt << "\t" << _desired_q_notcompensate(16) << "\t" << _desired_q_notcompensate(17) << "\t" << _desired_q_notcompensate(18) << "\t" << _desired_q_notcompensate(19)<< "\t" << _desired_q_notcompensate(20)<< "\t" << _desired_q_notcompensate(21)<< "\t" << _Gyro_Base[0]<< "\t" << _Gyro_Base[1] << endl;
        }
        file[1]<<_cnt<< "\t" << _L_Ft(0) << "\t" << _L_Ft(1) << "\t" << _L_Ft(2) << "\t" << _L_Ft(3)<< "\t" << _L_Ft(4)<< "\t" << _L_Ft(5)<< "\t" << _R_Ft(0)<< "\t" << _R_Ft(1)<< "\t" << _R_Ft(2)<< "\t" << _R_Ft(3)<< "\t" << _R_Ft(4)<< "\t" << _R_Ft(5) << endl;
        // fprintf(fp13,"%i\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_cnt,_T_Trunk_support_euler(0),_T_Trunk_support_euler(1),_T_Trunk_support_euler(2),trunk_temp(0),trunk_temp(1),trunk_temp(2),Foot_trajectory.LFoot.translation()(0),Foot_trajectory.LFoot.translation()(1),Foot_trajectory.LFoot.translation()(2));

        //Resolved_momentum_control();

        //Impedance_reference_update();

        VectorXD qd;
        qd.resize(12);
        qd.setZero();

        InverseKinematics(Trunk_trajectory_global.translation(),Foot_trajectory_global.RFoot.translation(),Foot_trajectory_global.LFoot.translation(),Trunk_trajectory_global.linear(),Foot_trajectory_global.RFoot.linear(),Foot_trajectory_global.LFoot.linear(),qd);

        //Impedance_reference_update();

        for(int i=0; i<6; i++)
        {
        _desired_q(i+RF_BEGIN)=qd(i);
        _desired_q(i+LF_BEGIN)=qd(i+6);
        }
        for (int i=0; i<7; i++)
        {
        _desired_q(RA_BEGIN+i) = _init_info.q(RA_BEGIN+i);
        _desired_q(LA_BEGIN+i) = _init_info.q(LA_BEGIN+i);
        }
        _desired_q(0) = _init_info.q(0);
        _desired_q(1) = _init_info.q(1);

        _desired_q_notcompensate=_desired_q;


        //hip_compensator();
        Hipcompensation();



        output = _desired_q;

        Step_time_update();

        Vector3D trunk_euler_desired;
        Matrix3D trunk_linear;
        trunk_linear = Trunk_trajectory.linear();
        Rot2euler(trunk_linear,trunk_euler_desired);

        _COM_update_flag = false;

        if(_cnt == _T_Start)
         ROS_INFO("%d %d",_cnt,_step_number);


        _cnt++;
    }
    else
    {
        output = _desired_q;
    }

}

void WalkingCtrl::setApproachdata(double x, double y, double theta)
{
    if(abs(x) < 0.01)
        _scan_data(0) = 0.0;
    else
        _scan_data(0) = x;

    if(abs(y) < 0.01)
        _scan_data(1) = 0.0;
    else
        _scan_data(1) = y;

    if(abs(theta) < 1*DEGREE)
        _scan_data(2) = 0.0;
    else
        _scan_data(2) = theta*DEGREE;

    _Step_Planning_flag = false;

    if( abs(x) < 0.01 || abs(y) < 0.01)
        _Step_Planning_flag = true;

  /*  if(abs(x) < 0.01)
    {
        _T_Double1 = 0.1*Hz;
        _T_Double2 = 0.1*Hz;

        _T_Total = 4.0*Hz;

        _T_temp = 3.0*Hz;
        _T_Imp = 0.0*Hz;

        _T_rest_init = 1.5*Hz;
        _T_rest_last = 1.5*Hz;

    }
*/

    _T_Last = _T_Total+_T_temp;
    _T_Start = _T_temp+1;
    _T_Start_real = _T_Start+_T_rest_init;
}


void WalkingCtrl::hip_compensator()
{

    double Left_Hip_angle = 3.8*DEGREE;
    double Right_Hip_angle = 3.15*DEGREE;
    double Left_Hip_angle_first_step = 3.8*DEGREE;
    double Right_Hip_angle_first_step = 3.15*DEGREE;

    double Left_hip_angle_temp = 0.0;
    double Right_hip_angle_temp = 0.0;
    double temp_time = 0.1*Hz;

    if(_step_number >= 3)
	Right_Hip_angle = 3.3*DEGREE;
	
    if(_step_number == 0)
    {
        if(_foot_step(_step_number,6) == 1) // �޹�����
        {
            if(_cnt < _T_Start +_T_Total-_T_rest_last-_T_Double2-temp_time)
                Left_hip_angle_temp = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+temp_time,0.0*DEGREE,0.0,Left_Hip_angle_first_step,0.0);
            else if(_cnt >= _T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time)
                Left_hip_angle_temp = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time,_T_Start+_T_Total-_T_rest_last,Left_Hip_angle_first_step,0.0,0.0,0.0);
            else
                Left_hip_angle_temp = 0.0*DEGREE;
        }
        else if(_foot_step(_step_number,6) == 0)
        {
            if(_cnt < _T_Start +_T_Total-_T_rest_last-_T_Double2-temp_time)
                Right_hip_angle_temp = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+temp_time,0.0*DEGREE,0.0,Right_Hip_angle_first_step,0.0);
            else if(_cnt >= _T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time)
                Right_hip_angle_temp = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time,_T_Start+_T_Total-_T_rest_last,Right_Hip_angle_first_step,0.0,0.0,0.0);
            else
                Right_hip_angle_temp = 0.0*DEGREE;
        }
        else
        {
            Left_hip_angle_temp = 0.0;
            Right_hip_angle_temp = 0.0*DEGREE;
        }
    }
    else
    {
        if(_foot_step(_step_number,6) == 1) // �޹�����
        {
            if(_cnt < _T_Start +_T_Total-_T_rest_last-_T_Double2-temp_time)
                Left_hip_angle_temp = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+temp_time,0.0*DEGREE,0.0,Left_Hip_angle,0.0);
            else if(_cnt >= _T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time)
                Left_hip_angle_temp = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time,_T_Start+_T_Total-_T_rest_last,Left_Hip_angle,0.0,0.0,0.0);
            else
                Left_hip_angle_temp = 0.0*DEGREE;
        }
        else if(_foot_step(_step_number,6) == 0)
        {
            if(_cnt < _T_Start +_T_Total-_T_rest_last-_T_Double2-temp_time)
                Right_hip_angle_temp = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+temp_time,0.0*DEGREE,0.0,Right_Hip_angle,0.0);
            else if(_cnt >= _T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time)
                Right_hip_angle_temp = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-temp_time,_T_Start+_T_Total-_T_rest_last,Right_Hip_angle,0.0,0.0,0.0);
            else
                Right_hip_angle_temp = 0.0*DEGREE;
        }
        else
        {
            Left_hip_angle_temp = 0.0;
            Right_hip_angle_temp = 0.0*DEGREE;
        }
    }

    _desired_q(RF_BEGIN+1) = _desired_q(RF_BEGIN+1) - Right_hip_angle_temp;
    _desired_q(LF_BEGIN+1) = _desired_q(LF_BEGIN+1) + Left_hip_angle_temp;
}


void WalkingCtrl::Step_time_update()
{
    if(_cnt == _T_Last && _step_number != _step_total_number-1)
    {

        _T_Start = _T_Last +1;
        _T_Start_real = _T_Start + _T_rest_init;
        _T_Last = _T_Start + _T_Total -1;
        _Impedance_flag = false;
    }
}

void WalkingCtrl::Step_count(MatrixXD& foot_planning)
{
    if (_step_total_number == 1)
    {
        _step_number = 0;
    }
    else if (_step_total_number > 1)
    {
        if (_step_number < _step_total_number-1)
        {
            if (_cnt == _T_Start && _cnt > _T_temp+_T_Double1)
            {
                _step_number++;
                //cout << "step count" << _step_number << endl;
                //cout << "step count" << foot_planning(_step_number,6) << endl;
                //_T_Double = _T_Double_init;
                //_T_Total = _T_Total_init;
                //_T_Imp = _T_Imp_init;
                //_T_Rec = _T_Rec_init;

            }
        }
    }
}


void WalkingCtrl::Hipcompensation()
{
    double a_total=-0.0012;
    double b_total=0.00087420;
    double alpha;   // left foot weighting factor
    double alpha_1; // right foot weighting factor

    double rq0= _desired_q(18-2);
    double rq1= _desired_q(19-2);
    double rq2= _desired_q(20-2);
    double rq3= _desired_q(21-2);
    double rq4= _desired_q(22-2);
    double rq5= _desired_q(23-2);

    double lq0= _desired_q(24-2);
    double lq1= _desired_q(25-2);
    double lq2= _desired_q(26-2);
    double lq3= _desired_q(27-2);
    double lq4= _desired_q(28-2);
    double lq5= _desired_q(29-2);

    double robotweightforce=54.592*9.81;

    double fromright = cos(rq5)*sin(rq1)*1.829E-1+cos(rq4)*sin(rq5)*(3.0/1.0E1)+cos(rq0)*(cos(rq1)*cos(rq5)+sin(rq1)*(cos(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5))+sin(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5))))*(2.1E1/2.0E2)-cos(rq1)*(cos(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5))+sin(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5)))*1.829E-1+sin(rq0)*(cos(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5))-sin(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5)))*(2.1E1/2.0E2)-sin(rq3)*sin(rq4)*sin(rq5)*(3.0/1.0E1)+cos(rq3)*cos(rq4)*sin(rq5)*(3.0/1.0E1);
    double fromleft = cos(lq5)*sin(lq1)*1.829E-1+cos(lq4)*sin(lq5)*(3.0/1.0E1)-cos(lq0)*(cos(lq1)*cos(lq5)+sin(lq1)*(cos(lq2)*(sin(lq3)*sin(lq4)*sin(lq5)-cos(lq3)*cos(lq4)*sin(lq5))+sin(lq2)*(cos(lq3)*sin(lq4)*sin(lq5)+cos(lq4)*sin(lq3)*sin(lq5))))*(2.1E1/2.0E2)-cos(lq1)*(cos(lq2)*(sin(lq3)*sin(lq4)*sin(lq5)-cos(lq3)*cos(lq4)*sin(lq5))+sin(lq2)*(cos(lq3)*sin(lq4)*sin(lq5)+cos(lq4)*sin(lq3)*sin(lq5)))*1.829E-1+sin(lq0)*(cos(lq2)*(cos(lq3)*sin(lq4)*sin(lq5)+cos(lq4)*sin(lq3)*sin(lq5))-sin(lq2)*(sin(lq3)*sin(lq4)*sin(lq5)-cos(lq3)*cos(lq4)*sin(lq5)))*(2.1E1/2.0E2)-sin(lq3)*sin(lq4)*sin(lq5)*(3.0/1.0E1)+cos(lq3)*cos(lq4)*sin(lq5)*(3.0/1.0E1);

    fromright=fromright-0.5*0.17;
    fromleft=fromleft+0.5*0.17;

    alpha = -fromleft/(fromright-fromleft);

    //if(alpha>=1)
    //{
    //	alpha=1;
    //}
    //if(alpha=<0)
    //{
    //	alpha=0;
    //}

    if(fromright<=0)
    {
        alpha=1;
    }
    if(fromleft>=0)
    {
        alpha=0;
    }

    //alpha=0.5;
    //alpha=(cos(rq5)*sin(rq1)*1.829E-1+cos(rq4)*sin(rq5)*(3.0/1.0E1)+cos(rq0)*(cos(rq1)*cos(rq5)+sin(rq1)*(cos(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5))+sin(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5))))*(2.1E1/2.0E2)-cos(rq1)*(cos(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5))+sin(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5)))*1.829E-1+sin(rq0)*(cos(rq2)*(cos(rq3)*sin(rq4)*sin(rq5)+cos(rq4)*sin(rq3)*sin(rq5))-sin(rq2)*(sin(rq3)*sin(rq4)*sin(rq5)-cos(rq3)*cos(rq4)*sin(rq5)))*(2.1E1/2.0E2)-sin(rq3)*sin(rq4)*sin(rq5)*(3.0/1.0E1)+cos(rq3)*cos(rq4)*sin(rq5)*(3.0/1.0E1))/0.21;
    alpha_1=1-alpha;

    double fc_r=robotweightforce*alpha;
    double fc_l=robotweightforce*alpha_1;

    double mc_r=((abs(fromright)+0.5*0.17)*fc_r-(abs(fromleft)+0.5*0.17)*fc_l)*alpha_1;
    double mc_l=((abs(fromright)+0.5*0.17)*fc_r-(abs(fromleft)+0.5*0.17)*fc_l)*alpha;

    //double Jc2[6];
    //double Jc3[6];

    //double Jc8[6];
    //double Jc9[6];

    Vector6D Jc2;
    Vector6D Jc3;
    Vector6D Jc8;
    Vector6D Jc9;
    Jc2.setZero();
    Jc3.setZero();
    Jc8.setZero();
    Jc9.setZero();

    Jc2(0) = 0;
    Jc2(1) = cos(rq2)*sin(rq1)*(3.0/1.0E1)-sin(rq1)*sin(rq2)*sin(rq3)*(3.0/1.0E1)+cos(rq2)*cos(rq3)*sin(rq1)*(3.0/1.0E1);
    Jc2(2) = cos(rq1)*sin(rq2)*(3.0/1.0E1)+cos(rq1)*cos(rq2)*sin(rq3)*(3.0/1.0E1)+cos(rq1)*cos(rq3)*sin(rq2)*(3.0/1.0E1);
    Jc2(3) = cos(rq1)*cos(rq2)*sin(rq3)*(3.0/1.0E1)+cos(rq1)*cos(rq3)*sin(rq2)*(3.0/1.0E1);
    Jc2(4) = 0;
    Jc2(5) = 0;

    Jc3(0) = 0;
    Jc3(1) = cos(rq0);
    Jc3(2) = -cos(rq1)*sin(rq0);
    Jc3(3) = -cos(rq1)*sin(rq0);
    Jc3(4) = -cos(rq1)*sin(rq0);
    Jc3(5) = cos(rq4)*(cos(rq3)*(cos(rq0)*cos(rq2)-sin(rq0)*sin(rq1)*sin(rq2))-sin(rq3)*(cos(rq0)*sin(rq2)+cos(rq2)*sin(rq0)*sin(rq1)))-sin(rq4)*(cos(rq3)*(cos(rq0)*sin(rq2)+cos(rq2)*sin(rq0)*sin(rq1))+sin(rq3)*(cos(rq0)*cos(rq2)-sin(rq0)*sin(rq1)*sin(rq2)));

    Jc8(0) = 0;
    Jc8(1) = cos(lq2)*sin(lq1)*(3.0/1.0E1)+cos(lq2)*cos(lq3)*sin(lq1)*(3.0/1.0E1)-sin(lq1)*sin(lq2)*sin(lq3)*(3.0/1.0E1);
    Jc8(2) = cos(lq1)*sin(lq2)*(3.0/1.0E1)+cos(lq1)*cos(lq2)*sin(lq3)*(3.0/1.0E1)+cos(lq1)*cos(lq3)*sin(lq2)*(3.0/1.0E1);
    Jc8(3) = cos(lq1)*cos(lq2)*sin(lq3)*(3.0/1.0E1)+cos(lq1)*cos(lq3)*sin(lq2)*(3.0/1.0E1);
    Jc8(4) = 0;
    Jc8(5) = 0;

    Jc9(0) = 0;
    Jc9(1) = cos(lq0);
    Jc9(2) = cos(lq1)*sin(lq0);
    Jc9(3) = cos(lq1)*sin(lq0);
    Jc9(4) = cos(lq1)*sin(lq0);
    Jc9(5) = cos(lq4)*(cos(lq3)*(cos(lq0)*cos(lq2)+sin(lq0)*sin(lq1)*sin(lq2))-sin(lq3)*(cos(lq0)*sin(lq2)-cos(lq2)*sin(lq0)*sin(lq1)))-sin(lq4)*(cos(lq3)*(cos(lq0)*sin(lq2)-cos(lq2)*sin(lq0)*sin(lq1))+sin(lq3)*(cos(lq0)*cos(lq2)+sin(lq0)*sin(lq1)*sin(lq2)));



    //double lTau[6];
    //double rTau[6];

    Vector6D lTau;
    Vector6D rTau;
    lTau.setZero();
    rTau.setZero();

    rTau(0) = 0;
    rTau(1) = cos(rq2)*sin(rq1)*(-2.6960823E1)+sin(rq1)*sin(rq2)*sin(rq3)*1.6972281E1-cos(rq2)*cos(rq3)*sin(rq1)*1.6972281E1;
    rTau(2) = cos(rq1)*sin(rq2)*(-2.6960823E1)+pow(cos(rq0),2.0)*cos(rq1)*3.6112125645+cos(rq1)*pow(sin(rq0),2.0)*3.6112125645-cos(rq1)*cos(rq2)*sin(rq3)*1.6972281E1-cos(rq1)*cos(rq3)*sin(rq2)*1.6972281E1;
    rTau(3) = pow(cos(rq0),2.0)*cos(rq1)*3.2487791715+cos(rq1)*pow(sin(rq0),2.0)*3.2487791715-cos(rq1)*cos(rq2)*sin(rq3)*1.6972281E1-cos(rq1)*cos(rq3)*sin(rq2)*1.6972281E1;
    rTau(4) = pow(cos(rq0),2.0)*cos(rq1)*2.0451598605+cos(rq1)*pow(sin(rq0),2.0)*2.0451598605;
    rTau(5) = cos(rq0)*(cos(rq4)*(cos(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))-sin(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1)))-sin(rq4)*(cos(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1))+sin(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))))*3.62433393E-1-sin(rq0)*(cos(rq4)*(cos(rq3)*(cos(rq0)*cos(rq2)-sin(rq0)*sin(rq1)*sin(rq2))-sin(rq3)*(cos(rq0)*sin(rq2)+cos(rq2)*sin(rq0)*sin(rq1)))-sin(rq4)*(cos(rq3)*(cos(rq0)*sin(rq2)+cos(rq2)*sin(rq0)*sin(rq1))+sin(rq3)*(cos(rq0)*cos(rq2)-sin(rq0)*sin(rq1)*sin(rq2))))*3.62433393E-1;

    lTau(0) = 0;
    lTau(1) = cos(lq2)*sin(lq1)*(-2.0857041E1)-cos(lq2)*cos(lq3)*sin(lq1)*6.892506+sin(lq1)*sin(lq2)*sin(lq3)*6.892506;
    lTau(2) = cos(lq1)*sin(lq2)*(-2.0857041E1)-pow(cos(lq0),2.0)*cos(lq1)*3.7168927515-cos(lq1)*pow(sin(lq0),2.0)*3.7168927515-cos(lq1)*cos(lq2)*sin(lq3)*6.892506-cos(lq1)*cos(lq3)*sin(lq2)*6.892506;
    lTau(3) = pow(cos(lq0),2.0)*cos(lq1)*(-2.5132734405)-cos(lq1)*pow(sin(lq0),2.0)*2.5132734405-cos(lq1)*cos(lq2)*sin(lq3)*6.892506-cos(lq1)*cos(lq3)*sin(lq2)*6.892506;
    lTau(4) = pow(cos(lq0),2.0)*cos(lq1)*(-8.30546973E-1)-cos(lq1)*pow(sin(lq0),2.0)*8.30546973E-1;
    lTau(5) = cos(lq0)*(cos(lq4)*(cos(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))-sin(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1)))-sin(lq4)*(cos(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1))+sin(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))))*4.6811358E-1-sin(lq0)*(cos(lq4)*(cos(lq3)*(cos(lq0)*cos(lq2)+sin(lq0)*sin(lq1)*sin(lq2))-sin(lq3)*(cos(lq0)*sin(lq2)-cos(lq2)*sin(lq0)*sin(lq1)))-sin(lq4)*(cos(lq3)*(cos(lq0)*sin(lq2)-cos(lq2)*sin(lq0)*sin(lq1))+sin(lq3)*(cos(lq0)*cos(lq2)+sin(lq0)*sin(lq1)*sin(lq2))))*4.6811358E-1;

    for(int i=0; i<6; i++)
    {
        rTau(i)=rTau(i)+Jc2(i)*fc_r;//+Jc3(i)*mc_r;
        lTau(i)=lTau(i)+Jc8(i)*fc_l;//+Jc9(i)*mc_l;
    }
    //_desired_q(18)=_desired_q(18)+(a_total*rTau[0]+b_total);
    //_desired_q(19-2)=_desired_q(19-2)+(a_total*rTau(1)+b_total);
    //_desired_q(20-2)=_desired_q(20-2)+(a_total*rTau(2)+b_total);
    //_desired_q(21-2)=_desired_q(21-2)+(a_total*rTau(3)+b_total);
    //_desired_q(22-2)=_desired_q(22-2)+(a_total*rTau(4)+b_total);
    //_desired_q(23-2)=_desired_q(23-2)+(a_total*rTau(5)+b_total);

    //_desired_q(24)=_desired_q(24)+(a_total*lTau[0]+b_total);
    //_desired_q(25-2)=_desired_q(25-2)+(a_total*lTau(1)+b_total);
    //_desired_q(26-2)=_desired_q(26-2)+(a_total*lTau(2)+b_total);
    //_desired_q(27-2)=_desired_q(27-2)+(a_total*lTau(3)+b_total);
    //_desired_q(28-2)=_desired_q(28-2)+(a_total*lTau(4)+b_total);
    //_desired_q(29-2)=_desired_q(29-2)+(a_total*lTau(5)+b_total);

    double rising=0;

    double timingtiming =  1;

    if(_cnt>_T_Start+_T_rest_init && _cnt<= _T_Start+_T_rest_init+_T_Double1*timingtiming)
    {
        rising = (_cnt-_T_Start-_T_rest_init)/(_T_Double1*timingtiming);
    }
    else if(_cnt> _T_Start+_T_rest_init+_T_Double1*timingtiming && _cnt<= _T_Start+_T_Total-_T_rest_last-_T_Double2*timingtiming )
    {
        rising =1;
    }
    else if( _cnt> _T_Start+_T_Total-_T_rest_last-_T_Double2*timingtiming &&  _cnt<= _T_Start+_T_Total-_T_rest_last)
    {
        rising = -(_cnt- (_T_Start+_T_Total-_T_rest_last))/(_T_Double2*timingtiming);
    }

    //rising=rising*1.0;
    rising=0.6;

    //_desired_q(18)=_desired_q(18)+(a_total*rTau[0]+b_total);
    //_desired_q(19)=_desired_q(19)+(a_total*rTau[1]+b_total)*rising;
    //_desired_q(20)=_desired_q(20)+(a_total*rTau[2]+b_total)*rising;
    //_desired_q(21)=_desired_q(21)+(a_total*rTau[3]+b_total)*rising;
    //_desired_q(22)=_desired_q(22)+(a_total*rTau[4]+b_total)*rising;
    //_desired_q(23)=_desired_q(23)+(a_total*rTau[5]+b_total)*rising;

    //_desired_q(24)=_desired_q(24)+(a_total*lTau[0]+b_total);
    //_desired_q(25)=_desired_q(25)+(a_total*lTau[1]+b_total)*rising;
    //_desired_q(26)=_desired_q(26)+(a_total*lTau[2]+b_total)*rising;
    //_desired_q(27)=_desired_q(27)+(a_total*lTau[3]+b_total)*rising;
    //_desired_q(28)=_desired_q(28)+(a_total*lTau[4]+b_total)*rising;
    //_desired_q(29)=_desired_q(29)+(a_total*lTau[5]+b_total)*rising;

    //_desired_q(18)=_desired_q(18)+(a_total*rTau[0]+b_total);
    _desired_q(19-2)=_desired_q(19-2)+(a_total*rTau(1)+b_total)*rising;
    //_desired_q(20-2)=_desired_q(20-2)+(a_total*rTau(2)+b_total)*rising;//offwhenslow
    //_desired_q(21-2)=_desired_q(21-2)+(a_total*rTau(3)+b_total)*rising*0.3;//offwhenslow
    //_desired_q(22-2)=_desired_q(22-2)+(a_total*rTau(4)+b_total)*rising;//offwhenslow
    _desired_q(23-2)=_desired_q(23-2)+(a_total*rTau(5)+b_total)*rising;

    //_desired_q(24)=_desired_q(24)+(a_total*lTau[0]+b_total);
    _desired_q(25-2)=_desired_q(25-2)+(a_total*lTau(1)+b_total)*rising;
    //_desired_q(26-2)=_desired_q(26-2)+(a_total*lTau(2)+b_total)*rising;//offwhenslow
    //_desired_q(27-2)=_desired_q(27-2)+(a_total*lTau(3)+b_total)*rising*0.3;//offwhenslow
    //_desired_q(28-2)=_desired_q(28-2)+(a_total*lTau(4)+b_total)*rising;//offwhenslow
    _desired_q(29-2)=_desired_q(29-2)+(a_total*lTau(5)+b_total)*rising;


}

void WalkingCtrl::_init_state_update()
{
    if(_cnt == 0)
    {
        _init_Foot_trajectory.LFoot.translation() = _T_LFoot_support[5].translation();
        _init_Foot_trajectory.LFoot.linear() = _T_LFoot_support[5].linear();
        _init_Foot_trajectory.LFoot_euler = _T_LFoot_support_euler;

        _init_Foot_trajectory.RFoot.translation() = _T_RFoot_support[5].translation();
        _init_Foot_trajectory.RFoot.linear() = _T_RFoot_support[5].linear();
        _init_Foot_trajectory.RFoot_euler = _T_RFoot_support_euler;


        _init_COM._COM = _COM_real_support;
        _init_COM._Trunk.translation() = _T_Trunk_support.translation();
        _init_COM._Trunk.linear() = _T_Trunk_support.linear();
        _init_COM._Trunk_euler = _T_Trunk_support_euler;
    }
    if(_step_number != 0)
    {
        if(_cnt == _T_Start || _COM_update_flag == true)
        {
            _init_Foot_trajectory.LFoot.translation() = _T_LFoot_support[5].translation();
            _init_Foot_trajectory.LFoot.linear() = _T_LFoot_support[5].linear();
            _init_Foot_trajectory.LFoot_euler = _T_LFoot_support_euler;

            _init_Foot_trajectory.RFoot.translation() = _T_RFoot_support[5].translation();
            _init_Foot_trajectory.RFoot.linear() = _T_RFoot_support[5].linear();
            _init_Foot_trajectory.RFoot_euler = _T_RFoot_support_euler;

            _init_COM._COM = _COM_real_support;
            _init_COM._Trunk.translation() = _T_Trunk_support.translation();
            _init_COM._Trunk.linear() = _T_Trunk_support.linear();
            _init_COM._Trunk_euler = _T_Trunk_support_euler;
        }
    }
}

void WalkingCtrl::InverseKinematics(Vector3D P_wt, Vector3D P_wr5, Vector3D P_wl5, Matrix3D R_wt, Matrix3D R_wr5, Matrix3D R_wl5, VectorXD& qd)
//void WalkingCtrl::InverseKinematics(Vector3D lr, Vector3D rr, Matrix3D R_wt, Matrix3D R_wr5, Matrix3D R_wl5, VectorXD& qd)
{

    // P_wt : w(world frame)¿¡Œ­ ¹Ù¶óº» trunk frameÀÇ position
    // P_wr5 : w(world frame)¿¡Œ­ ¹Ù¶óº» R[5] frameÀÇ position
    // P_wl5 : w(world frame)¿¡Œ­ ¹Ù¶óº» L[5] frameÀÇ position
    // R_wt : w(world frame)¿¡Œ­ ¹Ù¶óº» trunk frameÀÇ orientation
    // R_wr5 : w(world frame)¿¡Œ­ ¹Ù¶óº» R[5] frameÀÇ orientation
    // R_wl5 : w(world frame)¿¡Œ­ ¹Ù¶óº» L[5] frameÀÇ orientation

    //////////////// [5] frameÀÌ ankle¿¡ ºÙŸîÀÖŽÙ°í »ý°¢ !!!!!!!! (¹ß¹ÙŽÚ ŸÆŽÔ)
    // qd : 0~5 - right, 6~11 - left
    // °üÀýºÎÈ£ È®ÀÎ !!!!!!!!

    // °¢ ¹ß¿¡Œ­ º» hip±îÁöÀÇ local vector·Î º¯È¯
    Vector3D lp; // ¹ß¿¡Œ­ º» trunk±îÁö local vector
    Vector3D rp;

    lp.setZero();
    rp.setZero();

    Matrix3D R_wl5_trans;
    R_wl5_trans= R_wl5.transpose();
    Matrix3D R_wr5_trans;
    R_wr5_trans = R_wr5.transpose();
    Matrix3D R_wt_trans;
    R_wt_trans =R_wt.transpose();

    lp = R_wl5_trans*(P_wt-P_wl5);
    rp = R_wr5_trans*(P_wt-P_wr5);

    Matrix3D R_tl5;
    Matrix3D R_tr5;
    Matrix3D R_tl5_trans;
    Matrix3D R_tr5_trans;

    R_tl5.setZero();
    R_tr5.setZero();
    R_tl5_trans.setZero();
    R_tr5_trans.setZero();
    R_tl5 = R_wt_trans*R_wl5;
    R_tr5 = R_wt_trans*R_wr5;
    R_tl5_trans = R_tl5.transpose();
    R_tr5_trans =R_tr5.transpose();

    Vector3D ld; // trunk¿¡Œ­ º» hip([2] frame)±îÁö local vector
    Vector3D rd;

    ld.setZero();
    rd.setZero();

    ld(1) = 0.105;	/////////////////// THORMANG¿¡ žÂ°Ô x¹× zµµ ŒöÁ€ !!!!!!
    ld(2) = -0.1829;
    rd(1) = -0.105;
    rd(2) = -0.1829;
    ld = R_tl5_trans*ld;
    rd = R_tr5_trans*rd;

    Vector3D lr; // ¹ß¿¡Œ­ º» hip±îÁö local vector
    Vector3D rr;

    lr.setZero();
    rr.setZero();
    lr = lp + ld;
    rr = rp + rd;



    // link configuration /////////////////// THORMANG¿¡ žÂ°Ô ŒöÁ€ !!!!!!
    double l3 = 0.30; // Çã¹÷Áö ±æÀÌ
    double l4 = 0.30; // ÁŸŸÆž® ±æÀÌ
    double l5 = 0.10; // ¹ßžñ-¹ß ±æÀÌ

    ///////////////////////////////////// INVERSE KINEMATICS CALCULATION //////////////////////////////////////////

    ///////////////////////////////////// LEFT LEG /////////////////////////////////////

    double LC = sqrt(lr(0)*lr(0) + lr(1)*lr(1) + lr(2)*lr(2));
    qd(9) = (- acos((l3*l3 + l4*l4 - LC*LC) / (2*l3*l4)) + 3.141592);

    double lalp = asin((l3*sin(3.141592-qd(9)))/LC);
    qd(10) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - lalp;
    qd(11) = atan2(lr(1), lr(2));

    Matrix3D R_tl2;
    Matrix3D R_l2l3;
    Matrix3D R_l3l4;
    Matrix3D R_l4l5;

    R_tl2.setZero();
    R_l2l3.setZero();
    R_l3l4.setZero();
    R_l4l5.setZero();

    R_l2l3(0,0) = cos(qd(9));
    R_l2l3(0,2) = sin(qd(9));
    R_l2l3(1,1) = 1.0;
    R_l2l3(2,0) = -sin(qd(9));
    R_l2l3(2,2) = cos(qd(9));

    R_l3l4(0,0) = cos(qd(10));
    R_l3l4(0,2) = sin(qd(10));
    R_l3l4(1,1) = 1.0;
    R_l3l4(2,0) = -sin(qd(10));
    R_l3l4(2,2) = cos(qd(10));

    R_l4l5(0,0) = 1.0;
    R_l4l5(1,1) = cos(qd(11));
    R_l4l5(1,2) = -sin(qd(11));
    R_l4l5(2,1) = sin(qd(11));
    R_l4l5(2,2) = cos(qd(11));

    Matrix3D R_l4l5_t;
    Matrix3D R_l2l3_t;
    Matrix3D R_l3l4_t;
    R_l4l5_t = R_l4l5.transpose();
    R_l3l4_t = R_l3l4.transpose();
    R_l2l3_t = R_l2l3.transpose();

    R_tl2 = R_tl5 * R_l4l5_t * R_l3l4_t * R_l2l3_t;
    qd(7) = asin(R_tl2(2,1));

    double b=-R_tl2(0,1)/cos(qd(7));
    if (b > 1.0){

        b= 1.0;
    }
    else if ( b< -1.0)
    {

        b=-1.0;
    }

    qd(6) = -asin(b);
    //qd(6) = 0.0;
    //qd(8) = -acos(R_tl2(2,2)/cos(qd(7))); // ±âÁž °ª
    qd(8) = - asin(R_tl2(2,0)/cos(qd(7)));//+3.141592/2; //arcsineÀž·Î


    ///////////////////////////////////////// RIGHT LEG //////////////////////////////////////////

    float RC = sqrt(rr(0)*rr(0) + rr(1)*rr(1) + rr(2)*rr(2));
    qd(3) = (- acos((l3*l3 + l4*l4 - RC*RC) / (2*l3*l4)) + 3.141592);

    float ralp = asin((l3*sin(3.141592-qd(3)))/RC);

    qd(4) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2))) - ralp;
    qd(5) = atan2(rr(1), rr(2));

    Matrix3D R_tr2;
    Matrix3D R_r2r3;
    Matrix3D R_r3r4;
    Matrix3D R_r4r5;

    R_tr2.setZero();
    R_r2r3.setZero();
    R_r3r4.setZero();
    R_r4r5.setZero();

    R_r2r3(0,0) = cos(qd(3));
    R_r2r3(0,2) = sin(qd(3));
    R_r2r3(1,1) = 1.0;
    R_r2r3(2,0) = -sin(qd(3));
    R_r2r3(2,2) = cos(qd(3));

    R_r3r4(0,0) = cos(qd(4));
    R_r3r4(0,2) = sin(qd(4));
    R_r3r4(1,1) = 1.0;
    R_r3r4(2,0) = -sin(qd(4));
    R_r3r4(2,2) = cos(qd(4));

    R_r4r5(0,0) = 1.0;
    R_r4r5(1,1) = cos(qd(5));
    R_r4r5(1,2) = -sin(qd(5));
    R_r4r5(2,1) = sin(qd(5));
    R_r4r5(2,2) = cos(qd(5));

    Matrix3D R_r4r5_t;
    Matrix3D R_r2r3_t;
    Matrix3D R_r3r4_t;
    R_r4r5_t = R_r4r5.transpose();
    R_r3r4_t = R_r3r4.transpose();
    R_r2r3_t =R_r2r3.transpose();

    R_tr2 = R_tr5 * R_r4r5_t * R_r3r4_t * R_r2r3_t;

    qd(1) = asin(R_tr2(2,1));
    double a=-R_tr2(0,1)/cos(qd(1));
    if (a > 1.0)
    {

        a= 1.0;
    }
    else if ( a< -1.0)
    {

        a=-1.0;
    }

    qd(0) = -acos(R_tr2(1,1)/cos(qd(1)));
    qd(0) = -asin(a);
    //qd(0) = 0.0;
    //qd(2) = -acos(R_tr2(2,2)/cos(qd(1))); // ±âÁž °ª
    qd(2) = -asin(R_tr2(2,0)/cos(qd(1)));// - 3.141592/2 ; //arcsinÀž·Î



    // THORMANG °üÀý ºÎÈ£¿¡ žÂÃß±â
    qd(2) = -qd(2);
    qd(3) = -qd(3);
    qd(4) = -qd(4);

}

void WalkingCtrl::getdata(VectorXD& _q_robot, Vector6D& _L_Ft_robot, Vector6D& _R_Ft_robot, Vector3D& _Gyro_Base_robot)
{
    _q = _q_robot;
    _L_Ft = _L_Ft_robot;
    _R_Ft = _R_Ft_robot;
    _Gyro_Base = _Gyro_Base_robot;
    //_Gyro_Base.setZero();

    _L_FT_lowpass = _L_Ft;
    _R_FT_lowpass = _R_Ft;

}


void WalkingCtrl::update_robot_data()
{
    _q = _q;
    _L_Ft = _L_Ft;
    _R_Ft = _R_Ft;
    _Gyro_Base = _Gyro_Base;
    _Gyro_LFoot = _Gyro_LFoot;
    _Gyro_RFoot = _Gyro_RFoot;
}





void WalkingCtrl::_initialize()
{
    WALKINGpara_initialize();
    set_user_value();
    Robot_State_para_initialize();
    Pattern_generator_initialize();

    _T_Last = _T_Total+_T_temp;
    _T_Start = _T_temp+1;
    _T_Start_real = _T_Start+_T_rest_init;

    _desired_q_notcompensate.resize(28);
    _desired_q_notcompensate.setZero();
}
