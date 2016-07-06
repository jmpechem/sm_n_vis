#include "Walking_Controller.h"

WalkingCtrl::WalkingCtrl(){}
WalkingCtrl::~WalkingCtrl(){}

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


        Robot_state_update();

       // Vector3D Lfoot = _T_LFoot_support[5].translation();
       // Vector3D Rfoot = _T_RFoot_support[5].translation();
       // ZMP_real(_L_FT_global, _R_FT_global, Lfoot, Rfoot, _ZMP_real);


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
        file[12] << _cnt << "\t" << _T_Trunk_support.translation()(2) << "\t" << _COM_desired(2) << "\t" << Trunk_trajectory.translation()(2) << "\t" << _init_info._trunk_support_init.translation()(2) << endl;
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


        hip_compensator();



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

    if(abs(x) < 0.01)
    {
        _T_Double1 = 0.1*Hz;
        _T_Double2 = 0.1*Hz;

        _T_Total = 4.0*Hz;

        _T_temp = 3.0*Hz;
        _T_Imp = 0.0*Hz;

        _T_rest_init = 1.5*Hz;
        _T_rest_last = 1.5*Hz;

    }


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
}
