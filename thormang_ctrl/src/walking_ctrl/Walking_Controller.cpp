#include "Walking_Controller.h"

WalkingCtrl::WalkingCtrl(){};
WalkingCtrl::~WalkingCtrl(){};

void WalkingCtrl::Init_walking_pose(VectorXD& output)
{
    if(_cnt == 0)
    {
        _init_q = _q;
        cout <<"init_q" << _init_q << endl;
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
            Vector3D _scan_data;
            _scan_data.setZero();
            _scan_data(0) = 1.0;
            plan_foot_step(_scan_data,_foot_step,_Step_Planning_flag);
        }

        Step_count(_foot_step);

        Robot_state_update();
        //cout << _T_RArm_global[5].translation() << endl;
	cout << "1" << endl;
        Vector3D Lfoot = _T_LFoot_support[5].translation();
        Vector3D Rfoot = _T_RFoot_support[5].translation();
        ZMP_real(_L_FT_global, _R_FT_global, Lfoot, Rfoot, _ZMP_real);

	cout << "1" << endl;

        if(_cnt == _T_Start && _step_number != 0)
            _COM_update_flag = true;

	cout << "1" << endl;

        _init_state_update();
        if(_cnt == 0)
            Ref_ZMP_update();
        else if(_cnt == _T_Start && _step_number != 0)
            Ref_ZMP_update();

	cout << "1" << endl;
        Ref_COM_update_local();
	cout << "2" << endl;
        Trunk_trajectory_update();
	cout << "3" << endl;

        Foot_trajectory_update();
	cout << "4" << endl;
        Change_Global_Pattern();
	cout << "5" << endl;
        //Impedance_controller();
	cout << "6" << endl;
        Change_Local_Pattern();
	cout << "7" << endl;
        double k = 100.0;
        for (int i=0; i<3; i++)
        {
            if(_foot_step(_step_number,6) == 1) //¿Þ¹ßÁöÁö
            {
                Desired_RFoot_dot(i) = k*(Foot_trajectory.RFoot.translation()(i)-_T_RFoot_support[5].translation()(i));
                Desired_RFoot_dot(i+3) = k*(Foot_trajectory.RFoot_euler(i)-_T_RFoot_support_euler(i));
                //Desired_LFoot_dot.setZero();
                Desired_LFoot_dot(i) = k*(Foot_trajectory.LFoot.translation()(i)-_T_LFoot_support[5].translation()(i));
                Desired_LFoot_dot(i+3) = k*(Foot_trajectory.LFoot_euler(i)-_T_LFoot_support_euler(i));

            }
            else
            {
                Desired_LFoot_dot(i) = k*(Foot_trajectory.LFoot.translation()(i)-_T_LFoot_support[5].translation()(i));
                Desired_LFoot_dot(i+3) = k*(Foot_trajectory.LFoot_euler(i)-_T_LFoot_support_euler(i));
                //Desired_RFoot_dot.setZero();
                Desired_RFoot_dot(i) = k*(Foot_trajectory.RFoot.translation()(i)-_T_RFoot_support[5].translation()(i));
                Desired_RFoot_dot(i+3) = k*(Foot_trajectory.RFoot_euler(i)-_T_RFoot_support_euler(i));
            }

        }


        Vector3D trunk_temp;
        Rot2euler(Trunk_trajectory.linear(),trunk_temp);
cout << "8" << endl;
        //fprintf(fp13,"%i\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_cnt,_T_Trunk_support_euler(0),_T_Trunk_support_euler(1),_T_Trunk_support_euler(2),trunk_temp(0),trunk_temp(1),trunk_temp(2),Foot_trajectory.LFoot.translation()(0),Foot_trajectory.LFoot.translation()(1),Foot_trajectory.LFoot.translation()(2));
        //Trunk_trajectory.translation() = _init_COM._Trunk.translation();
        //Change_Global_Pattern();

        //Heel_Toe_Motion();
        //Heel_Toe_Motion_pattern();

        //Resolved_momentum_control();

        Impedance_reference_update();
cout << "9" << endl;
        VectorXD qd;
        qd.resize(12);
        qd.setZero();

        InverseKinematics(Trunk_trajectory_global.translation(),Foot_trajectory_global.RFoot.translation(),Foot_trajectory_global.LFoot.translation(),Trunk_trajectory_global.linear(),Foot_trajectory_global.RFoot.linear(),Foot_trajectory_global.LFoot.linear(),qd);
cout << "7" << endl;
        Impedance_reference_update();


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
        

        output = _desired_q;

        Step_time_update();

       // fprintf(fp1,"%f\t%f\t%f\t%f\n",_ZMP_desired(0),_ZMP_desired(1),_ZMP_real(0),_ZMP_real(1));
        //fprintf(fp2,"%f\t%f\t%f\t%f\t%f\t%f\n",_COM_desired(0),_COM_desired(1),_COM_desired(2),_COM_real_support(0),_COM_real_support(1),_COM_real_support(2));

        //fprintf(fp3,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_desired_q(0),_desired_q(1),_desired_q(2), _desired_q(3), _desired_q(4), _desired_q(5),_desired_q(6),_desired_q(7),_desired_q(8), _desired_q(10), _desired_q(11), _desired_q(12),_desired_q(13));
        //fprintf(fp3,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_desired_q(16),_desired_q(17),_desired_q(18), _desired_q(19), _desired_q(20), _desired_q(21),_desired_q(22),_desired_q(23),_desired_q(24), _desired_q(25), _desired_q(26), _desired_q(27));
        //fprintf(fp4,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_q(16),_q(17),_q(18), _q(19), _q(20), _q(21),_q(22),_q(23),_q(24), _q(25), _q(26), _q(27));

        //fprintf(fp5,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_COM_desired(0),_COM_desired(1),_COM_desired(2),Foot_trajectory.RFoot.translation()(0),Foot_trajectory.RFoot.translation()(1),Foot_trajectory.RFoot.translation()(2),Foot_trajectory.LFoot.translation()(0),Foot_trajectory.LFoot.translation()(1),Foot_trajectory.LFoot.translation()(2),_T_Trunk_support_euler(0),_T_Trunk_support_euler(1),_T_Trunk_support_euler(2),_T_RFoot_support_euler(0),_T_RFoot_support_euler(1),_T_RFoot_support_euler(2),_T_LFoot_support_euler(0),_T_LFoot_support_euler(1),_T_LFoot_support_euler(2));
        //fprintf(fp9,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_COM_real_support(0),_COM_real_support(1),_COM_real_support(2),_T_RFoot_support[5].translation()(0),_T_RFoot_support[5].translation()(1),_T_RFoot_support[5].translation()(2),_T_LFoot_support[5].translation()(0),_T_LFoot_support[5].translation()(1),_T_LFoot_support[5].translation()(2),_T_Trunk_support_euler(0),_T_Trunk_support_euler(1),_T_Trunk_support_euler(2),_T_RFoot_support_euler(0),_T_RFoot_support_euler(1),_T_RFoot_support_euler(2),_T_LFoot_support_euler(0),_T_LFoot_support_euler(1),_T_LFoot_support_euler(2));


        Vector3D trunk_euler_desired;
        Matrix3D trunk_linear;
        trunk_linear = Trunk_trajectory.linear();
        Rot2euler(trunk_linear,trunk_euler_desired);

        //fprintf(fp6,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",trunk_euler_desired(0),trunk_euler_desired(1),trunk_euler_desired(2),Foot_trajectory.RFoot_euler(0),Foot_trajectory.RFoot_euler(1),Foot_trajectory.RFoot_euler(2),Foot_trajectory.LFoot_euler(0),Foot_trajectory.LFoot_euler(1),Foot_trajectory.LFoot_euler(2));
        //fprintf(fp12,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_T_Trunk_support_euler(0),_T_Trunk_support_euler(1),_T_Trunk_support_euler(2),_T_RFoot_support_euler(0),_T_RFoot_support_euler(1),_T_RFoot_support_euler(2),_T_LFoot_support_euler(0),_T_LFoot_support_euler(1),_T_LFoot_support_euler(2));
        //fprintf(fp9,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_T_Trunk_support.translation()(0),_T_Trunk_support.translation()(1),_T_Trunk_support.translation()(2),_T_LFoot_support[5].translation()(0),_T_LFoot_support[5].translation()(1),_T_LFoot_support[5].translation()(2),_T_RFoot_support[5].translation()(0),_T_RFoot_support[5].translation()(1),_T_RFoot_support[5].translation()(2));
        //fprintf(fp9,"%f\t%f\t%f\t%f\t%f\t%f\n",_T_RFoot_global[5].translation()(0),_T_RFoot_global[5].translation()(1),_T_RFoot_global[5].translation()(2),_T_LFoot_global[5].translation()(0),_T_LFoot_global[5].translation()(1),_T_LFoot_global[5].translation()(2));
        //fprintf(fp10,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_Gyro_Base[0],_Gyro_Base[1],_Gyro_Base[2],Base_global_position[0],Base_global_position[1],Base_global_position(2),_LFoot_position[0],_LFoot_position[1],_LFoot_position[2]);

        //fprintf(fp7,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",_R_FT_global(0),_R_FT_global(1),_R_FT_global(2),_R_FT_global(3),_R_FT_global(4),_R_FT_global(5),_L_FT_global(0),_L_FT_global(1),_L_FT_global(2),_L_FT_global(3),_L_FT_global(4),_L_FT_global(5));
        _COM_update_flag = false;

        _cnt++;
    }

}

void WalkingCtrl::Step_time_update()
{
    if(_cnt == _T_Last && _step_number != _step_total_number-1)
    {
        _T_Start = _T_Last +1;
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
    qd(8) = -acos(R_tl2(2,2)/cos(qd(7)));


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
    qd(2) = -acos(R_tr2(2,2)/cos(qd(1)));


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
   // _Gyro_LFoot = _Gyro_LFoot_robot;
   // _Gyro_RFoot = _Gyro_RFoot_robot;
   // Base_global_position = _Base_position;
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
}
