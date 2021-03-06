#include "Robot_Controller.h"

void Robot_Control::Resolved_momentum_control()
{
    MatrixXD M_Lleg, M_Rleg, H_Lleg, H_Rleg;
    MatrixXD M_Lf, M_Rf, H_Lf, H_Rf;

    M_Lleg.resize(3,6); M_Lleg.setZero();
    M_Rleg.resize(3,6); M_Rleg.setZero();
    H_Lleg.resize(3,6); H_Lleg.setZero();
    H_Rleg.resize(3,6); H_Rleg.setZero();
    M_Lf.resize(3,6); M_Lf.setZero();
    M_Rf.resize(3,6); M_Rf.setZero();
    H_Lf.resize(3,6); H_Lf.setZero();
    H_Rf.resize(3,6); H_Rf.setZero();


    for (int i=0; i<6; i++)
    {
        for (int j=0; j<3; j++)
        {
            M_Lleg(j,i) = _Momentum_Mass(j,LF_BEGIN+i);
            M_Rleg(j,i) = _Momentum_Mass(j,RF_BEGIN+i);
            H_Lleg(j,i) = _Momentum_Inertia_com(j,LF_BEGIN+i);
            H_Rleg(j,i) = _Momentum_Inertia_com(j,RF_BEGIN+i);
        }
    }
    //cout << "mm" << _Momentum_Inertia_com << endl;
    Matrix6D J_Lfoot_inv, J_Rfoot_inv;
    J_Lfoot_inv = _J_LFoot_global[5].inverse();
    J_Rfoot_inv = _J_RFoot_global[5].inverse();

    M_Lf = M_Lleg*J_Lfoot_inv;
    M_Rf = M_Rleg*J_Rfoot_inv;
    H_Lf = H_Lleg*J_Lfoot_inv;
    H_Rf = H_Rleg*J_Rfoot_inv;


    /////////total mass, Inertia ���� ///////////////

    double total_mass = 0.0;
    for (int i=0; i<29; i++)
        total_mass += Mass(i);

    Matrix3D total_inertia;
    total_inertia.setZero();

    Vector3D com_displace;
    Matrix3D com_dis_matrix;

    com_displace = _Momentum_COM_hat[WA_BEGIN]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    total_inertia = _inertia_hat[WA_BEGIN] + mass_hat[WA_BEGIN] * com_dis_matrix.transpose()*com_dis_matrix;

    com_displace = _Momentum_COM_hat[RF_BEGIN]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    total_inertia += _inertia_hat[RF_BEGIN] + mass_hat[RF_BEGIN] * com_dis_matrix.transpose()*com_dis_matrix;

    com_displace = _Momentum_COM_hat[LF_BEGIN]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    total_inertia += _inertia_hat[LF_BEGIN] + mass_hat[LF_BEGIN] * com_dis_matrix.transpose()*com_dis_matrix;

    com_displace = _m_trunk[0]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    total_inertia += _T_Waist_global[0].linear()*_Inertia_link[28]*_T_Waist_global[0].linear().transpose() + Mass[28] * com_dis_matrix.transpose()*com_dis_matrix;

    //cout << "total" << total_inertia << endl;

/*
    Matrix3D temp_inertia;
    temp_inertia.setZero();

    for (int i=0; i<7; i++)
    {
        com_displace = _COM_Rarm[i]-_COM_real_global;
        Skew(com_displace,com_dis_matrix);
        temp_inertia += _T_RArm_global[i].linear()*_Inertia_link[RA_BEGIN+i]*_T_RArm_global[i].linear().transpose() + Mass[RA_BEGIN+i]*com_dis_matrix.transpose()*com_dis_matrix;

        com_displace = _COM_Larm[i]-_COM_real_global;
        Skew(com_displace,com_dis_matrix);
        temp_inertia += _T_LArm_global[i].linear()*_Inertia_link[LA_BEGIN+i]*_T_LArm_global[i].linear().transpose() + Mass[LA_BEGIN+i]*com_dis_matrix.transpose()*com_dis_matrix;
    }

    for (int i=0; i<6; i++)
    {
        com_displace = _COM_R[i]-_COM_real_global;
        Skew(com_displace,com_dis_matrix);
        temp_inertia += _T_RFoot_global[i].linear()*_Inertia_link[RF_BEGIN+i]*_T_RFoot_global[i].linear().transpose() + Mass[RF_BEGIN+i]*com_dis_matrix.transpose()*com_dis_matrix;

        com_displace = _COM_L[i]-_COM_real_global;
        Skew(com_displace,com_dis_matrix);
        temp_inertia += _T_LFoot_global[i].linear()*_Inertia_link[LF_BEGIN+i]*_T_LFoot_global[i].linear().transpose() + Mass[LF_BEGIN+i]*com_dis_matrix.transpose()*com_dis_matrix;
    }

    com_displace = _m_trunk[2]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    temp_inertia += _T_Waist_global[1].linear()*_Inertia_link[WA_BEGIN+1]*_T_Waist_global[1].linear().transpose() + Mass[WA_BEGIN+1]*com_dis_matrix.transpose()*com_dis_matrix;

    com_displace = _m_trunk[1]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    temp_inertia += _T_Waist_global[0].linear()*_Inertia_link[WA_BEGIN]*_T_Waist_global[0].linear().transpose() + Mass[WA_BEGIN]*com_dis_matrix.transpose()*com_dis_matrix;

    com_displace = _m_trunk[0]-_COM_real_global;
    Skew(com_displace,com_dis_matrix);
    temp_inertia += _T_Waist_global[0].linear()*_Inertia_link[28]*_T_Waist_global[0].linear().transpose() + Mass[28]*com_dis_matrix.transpose()*com_dis_matrix;

    cout << "temp_inertia" << temp_inertia << endl;*/
    ////////////////////////////////////////////////////////////

    p_ref = p_ref*total_mass;
    int index;
    VectorXD target_q;
    target_q.resize(28);
    target_q.setZero();
    index = 0;

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


    //target_q(1) = Cubic(_cnt,0.0,3*Hz,0.0,0.0,10*DEGREE,0.0);
    double k = 100.0;
    VectorXD q_dot_desired;
    q_dot_desired.resize(16);
    for(int i=0; i<16; i++)
        q_dot_desired(i) = k*(target_q(i)-_q(i));

    Vector6D base_dot_desired;
    for (int i=0; i<3; i++)
    {
        base_dot_desired(i) = 0.0;// p_ref(i)/total_mass;
    }

    Vector3D trunk_euler_desired;
    Matrix3D trunk_linear;
    trunk_linear = Trunk_trajectory.linear();
    Rot2euler(trunk_linear,trunk_euler_desired);

    k = 100.0;
    base_dot_desired(3) = k*(trunk_euler_desired(0)-_T_Trunk_support_euler(0));
    //cout << "Gyro" << _Gyro_Base << endl;
    base_dot_desired(4) = k*(trunk_euler_desired(1)-_T_Trunk_support_euler(1));

    //trunk_euler_desired(2) = Cubic(_cnt,0,0.0,3*Hz,0.0,10.0*DEGREE,0.0);
    base_dot_desired(5) = k*(trunk_euler_desired(2)-_T_Trunk_support_euler(2));
    //base_dot_desired.setZero();
    //base_dot_desired(3) = -k*_Gyro_Base[0];
    //cout << "Gyro" << _Gyro_Base << endl;
//	base_dot_desired(4) = -k*_Gyro_Base[1];
    /*cout << "desired trunk" << trunk_euler_desired(2) << endl;
    cout << "current trunk" << _T_Trunk_support_euler(2) << endl;
    cout << "desired trunk" << _q(22) << endl;*/
//	cout << "td" << trunk_euler_desired(2)<< endl;

    /////////////////////////Mb , hb ����//////////////////////////

    MatrixXD Mb, Hb;
    Mb.resize(3,6); Hb.resize(3,6);

    MatrixXD Mb1, Hb1;
    Mb1.resize(3,6); Hb1.resize(3,6);
    Mb1.setZero(); Hb1.setZero();

    Matrix3D temp_Iden;
    temp_Iden.setIdentity();

    Matrix3D skew_com;
    Skew(_COM_real_global,skew_com);

    for (int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {

            Mb1(i,j) = temp_Iden(i,j) * total_mass;
            Mb1(i,j+3) = -skew_com(i,j) * total_mass;

            Hb1(i,j+3) = total_inertia(i,j);
        }
    }

    MatrixXD Mb2_left, Mb2_right;
    Mb2_left.resize(6,6); Mb2_right.resize(6,6);
    Mb2_left.setIdentity();Mb2_right.setIdentity();

    Matrix3D skew_left_foot, skew_right_foot;
    Skew(_T_RFoot_global[5].translation(),skew_right_foot);
    Skew(_T_LFoot_global[5].translation(),skew_left_foot);

    for(int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            Mb2_left(i,j+3) = -skew_left_foot(i,j);
            Mb2_right(i,j+3) = -skew_right_foot(i,j);
        }
    }

    Mb = Mb1-M_Rf*Mb2_right-M_Lf*Mb2_left;
    Hb = Hb1-H_Rf*Mb2_right-H_Lf*Mb2_left;

    //cout << "hb1" << Hb1 << endl;

    ////////////Desired momentum �� ���� constraint ��� //////////////
    Vector3D ref_y_linear, ref_y_angular;

    ref_y_linear = p_ref-M_Rf*Desired_RFoot_dot-M_Lf*Desired_LFoot_dot;
    ref_y_angular = L_ref-H_Rf*Desired_RFoot_dot-H_Lf*Desired_LFoot_dot;
    //cout << "HRF" << H_Rf << endl;


    MatrixXD A_momentum;
    A_momentum.resize(6,22);

    index = 0;
    for (int i=0; i<6; i++)
    {
        for (int j=0; j<3; j++)
        {
            A_momentum(j,i) = Mb(j,i);
            A_momentum(j+3,i) = Hb(j,i);
        }
    }
    index = 6;
    for (int i=0; i<2; i++)
    {
        for (int j=0; j<3; j++)
        {
            A_momentum(j,index+i) = _Momentum_Mass(j,WA_BEGIN+i);
            A_momentum(j+3,index+i) = _Momentum_Inertia_com(j,WA_BEGIN+i);
        }
    }
    index += 2;
    for (int i=0; i<7; i++)
    {
        for (int j=0; j<3; j++)
        {
            A_momentum(j,index+i) = _Momentum_Mass(j,RA_BEGIN+i);
            A_momentum(j+3,index+i) = _Momentum_Inertia_com(j,RA_BEGIN+i);
        }
    }
    index+= 7;
    for (int i=0; i<7; i++)
    {
        for (int j=0; j<3; j++)
        {
            A_momentum(j,index+i) = _Momentum_Mass(j,LA_BEGIN+i);
            A_momentum(j+3,index+i) = _Momentum_Inertia_com(j,LA_BEGIN+i);
        }
    }
    index+= 7;
    Vector6D ref_y;
    for (int i=0; i<3; i++)
    {
        ref_y(i) = ref_y_linear(i);
        ref_y(i+3) = ref_y_angular(i);
    }
    //cout << ref_y << endl;
    ///////////selection matrix//////////////


    MatrixXD Selec_Matrix;
    Selec_Matrix.resize(4,6);
    Selec_Matrix.setZero();
    Selec_Matrix(0,0) = 1.0;
    Selec_Matrix(1,1) = 1.0;
    Selec_Matrix(2,2) = 1.0;
    Selec_Matrix(3,5) = 1.0;

    VectorXD y;
    y.resize(4);
    y.setZero();

    y = Selec_Matrix*ref_y;
    MatrixXD A;
    A.resize(4,22);
    A = Selec_Matrix*A_momentum;

    MatrixXD A_inv;
    A_inv.resize(22,4);
    MatrixXD A_inv_temp;
    A_inv_temp.resize(4,4);
    A_inv_temp = A*A.transpose();
    A_inv = A.transpose()*A_inv_temp.inverse();
    //cout << A_inv << endl;

    VectorXD target_y;
    target_y.resize(22);

    MatrixXD Iden;
    Iden.resize(22,22);
    Iden.setIdentity();

    VectorXD total_ref;
    total_ref.resize(22);

    for (int i=0;i<6; i++)
    {
        total_ref(i) = base_dot_desired(i);
    }
    for (int i=0; i<16;i++)
        total_ref(i+6) = q_dot_desired(i);

    target_y = A_inv*y+(Iden-A_inv*A)*total_ref;

    VectorXD temp;
    temp.resize(22);
    temp = A_inv*y;
    //cout << "A_inv" << temp(7) << endl;

    VectorXD q_dot_free;
    q_dot_free.resize(16);

    for (int i=0; i<16; i++)
        q_dot_free(i) = target_y(6+i);

    Vector6D base_dot_target;

    for(int i=0; i<6; i++)
        base_dot_target(i) = target_y(i);


    /////////////������ ������ ���ϱ�///////////////////////////

    Matrix6D B_f1;
    Matrix6D B_f2;
    B_f1.setZero();
    B_f2.setZero();
    Matrix3D Iden_3;
    Iden_3.setIdentity();

    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {

            B_f1(i,j) = Iden_3(i,j);
            B_f2(i,j) = Iden_3(i,j);

            B_f1(i+3,j+3) = Iden_3(i,j);
            B_f2(i+3,j+3) = Iden_3(i,j);

            Matrix3D basetofoot;
            Skew(_T_RFoot_global[5].translation(),basetofoot);

            B_f1(i,j+3) = -basetofoot(i,j);

            Skew(_T_LFoot_global[5].translation(),basetofoot);
            B_f2(i,j+3) = -basetofoot(i,j);
        }
    }
    //J_Rfoot_inv = _J_RFoot_global[5].transpose();
    //J_Lfoot_inv = _J_LFoot_global[5].transpose();
    Vector6D q_dot_R_leg;

    q_dot_R_leg = J_Rfoot_inv*(Desired_RFoot_dot-B_f1*base_dot_target);

    Vector6D q_dot_L_leg;
    q_dot_L_leg = J_Lfoot_inv*(Desired_LFoot_dot-B_f2*base_dot_target);


    Vector6D R_dot;
    R_dot = Desired_RFoot_dot-B_f1*base_dot_target;
    Vector6D L_dot;
    L_dot = Desired_LFoot_dot-B_f2*base_dot_target;
    //cout << "base target" << base_dot_target << endl;
    //cout << "Right" << endl;
    //cout << "Dot" << Foot_trajectory.RFoot_dot << endl;
    //cout << "base" << B_f1*base_dot_target << endl;
    //cout << "q_dot_R_leg" << q_dot_R_leg << endl;

    //cout << "Left" << endl;
    //cout << "Dot" << Desired_LFoot_dot << endl;
    //cout << "base" << B_f2*base_dot_target << endl;
    //cout << "q_dot_R_leg" << q_dot_L_leg << endl;

    //cout << "base" << base_dot_target << endl;

    for (int i = WA_BEGIN; i< RF_BEGIN; i++)
    {
        //cout << "_cnt" << _cnt << endl;
        _desired_q(i) = _q(i)+q_dot_free(i)*1.0/Hz;
    //	cout << _desired_q(i) << endl;
    }

    double gain;
    gain = 1.0;

    for(int i=RF_BEGIN; i<LF_BEGIN; i++)
        _desired_q(i) = _q(i)+gain*q_dot_R_leg(i-RF_BEGIN)*1.0/Hz;

    for(int i=LF_BEGIN; i<28; i++)
        _desired_q(i) = _q(i)+gain*q_dot_L_leg(i-LF_BEGIN)*1.0/Hz;


    VectorXD q_dot_calculate;
    q_dot_calculate.resize(28);

    for (int i=WA_BEGIN; i<RF_BEGIN; i++)
        q_dot_calculate(i) = q_dot_free(i);
    for(int i=RF_BEGIN; i<LF_BEGIN; i++)
        q_dot_calculate(i) = q_dot_R_leg(i-RF_BEGIN);
    for(int i=LF_BEGIN; i<28; i++)
        q_dot_calculate(i) = q_dot_L_leg(i-LF_BEGIN);

    Vector3D linear_Mo;
    Vector3D omega;
    for (int i=0; i<3; i++)
        omega(i) = base_dot_target(i+3);

    linear_Mo = _Momentum_Inertia_com*q_dot_calculate;
    linear_Mo += total_inertia*omega;
    //cout << "linear" << linear_Mo << endl;


    MatrixXD lleg_temp, rleg_temp;
    lleg_temp.resize(3,6); rleg_temp.resize(3,6);

    for (int i=0; i<6; i++)
    {
        for (int j=0; j<3; j++)
        {
            lleg_temp(j,i) = _Momentum_Mass(j,LF_BEGIN+i);
            rleg_temp(j,i) = _Momentum_Mass(j,RF_BEGIN+i);
        }
    }

    Matrix6D iden_rleg, iden_lleg;
    iden_lleg.setIdentity(), iden_rleg.setIdentity();
    for(int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            Vector3D foot_position_temp;
            Matrix3D foot_pos;
            foot_position_temp = _T_RFoot_global[5].translation();
            Skew(foot_position_temp,foot_pos);
            iden_rleg(i,j+3) = -foot_pos(i,j);


            foot_position_temp = _T_LFoot_global[5].translation();
            Skew(foot_position_temp,foot_pos);
            iden_lleg(i,j+3) = -foot_pos(i,j);

        }
    }
    linear_Mo += Mb1*base_dot_target-lleg_temp*J_Lfoot_inv*iden_lleg*base_dot_target-rleg_temp*J_Rfoot_inv*iden_rleg*base_dot_target;
    //cout << "linear" << linear_Mo << endl;
    //cout << "reflinear" << p_ref << endl;
    VectorXD target_temp;
    target_temp.resize(22);
    for (int i=0; i<6; i++)
        target_temp(i) =base_dot_target(i);
    for (int i=0; i<16; i++)
        target_temp(i+6) = q_dot_calculate(i);
    Vector6D linear_Mo1;
    linear_Mo1 = A_momentum*target_temp;
    //cout << "linear" << linear_Mo1 << endl;

    //cout << "RFOOT" << Desired_RFoot_dot(2) << endl;

   // fprintf(fp10,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",p_ref(0),p_ref(1),p_ref(2),ref_y_angular(0),ref_y_angular(1),ref_y_angular(2),ref_y(0),ref_y(1),ref_y(2),base_dot_desired(0),base_dot_desired(1),base_dot_desired(2),base_dot_desired(3),base_dot_desired(4),base_dot_desired(5),base_dot_target(0),base_dot_target(1),base_dot_target(2),base_dot_target(3),base_dot_target(4),base_dot_target(5),R_dot(1),L_dot(1));
}


void Robot_Control::Impedance_controller()
{
    Impedance_update();
    if(_foot_step(_step_number,6) != 2 && _landing_flag == true)
        Impedance_control();

}

void Robot_Control::Impedance_reference_update()
{
    if(_cnt == 0)
    {
        for (int i=0; i<3;i++)
        {
            _Impedance_Ref_current_L(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_current_L(i+3) = Foot_trajectory_global.LFoot_euler(i);
            _Impedance_Ref_prev1_L(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_prev1_L(i+3) = Foot_trajectory_global.LFoot_euler(i);
            _Impedance_Ref_prev2_L(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_prev2_L(i+3) = Foot_trajectory_global.LFoot_euler(i);

            _Impedance_Ref_current_R(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_current_R(i+3) = Foot_trajectory_global.RFoot_euler(i);
            _Impedance_Ref_prev1_R(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_prev1_R(i+3) = Foot_trajectory_global.RFoot_euler(i);
            _Impedance_Ref_prev2_R(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_prev2_R(i+3) = Foot_trajectory_global.RFoot_euler(i);
        }
    }
    else
    {
        for (int i=0; i<6; i++)
        {
            _Impedance_Ref_prev2_L(i) = _Impedance_Ref_prev1_L(i);
            _Impedance_Ref_prev2_R(i) = _Impedance_Ref_prev1_R(i);

            _Impedance_Ref_prev1_L(i) = _Impedance_Ref_current_L(i);
            _Impedance_Ref_prev1_R(i) = _Impedance_Ref_current_R(i);
        }

        for (int i=0; i<3; i++)
        {
            _Impedance_Ref_current_L(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_current_L(i+3) = Foot_trajectory_global.LFoot_euler(i);

            _Impedance_Ref_current_R(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_current_R(i+3) = Foot_trajectory_global.RFoot_euler(i);
        }
    }
}


void Robot_Control::Impedance_after_update()
{
    for (int i=0; i<6; i++)
    {
        _Impedance_Ref_prev2_L2(i) = _Impedance_Ref_prev1_L2(i);
        _Impedance_Ref_prev2_R2(i) = _Impedance_Ref_prev1_R2(i);
    }

    for (int i=0; i<3; i++)
    {
        _Impedance_Ref_prev1_L2(i) = Foot_trajectory_global.LFoot.translation()(i);//_T_LFoot_global[5].translation()(i);
        _Impedance_Ref_prev1_L2(i+3) = Foot_trajectory_global.LFoot_euler(i);//_T_LFoot_global_euler(i);

        _Impedance_Ref_prev1_R2(i) = Foot_trajectory_global.RFoot.translation()(i);//_T_RFoot_global[5].translation()(i);
        _Impedance_Ref_prev1_R2(i+3) = Foot_trajectory_global.RFoot_euler(i);//_T_RFoot_global_euler(i);
    }

}

void Robot_Control::Impedance_after_update_initialize()
{
    if(_cnt == 0)
    {
        for (int i=0; i<3;i++)
        {
            _Impedance_Ref_prev1_L2(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_prev1_L2(i+3) = Foot_trajectory_global.LFoot_euler(i);
            _Impedance_Ref_prev2_L2(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_Ref_prev2_L2(i+3) = Foot_trajectory_global.LFoot_euler(i);

            _Impedance_Ref_prev1_R2(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_prev1_R2(i+3) = Foot_trajectory_global.RFoot_euler(i);
            _Impedance_Ref_prev2_R2(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_Ref_prev2_R2(i+3) = Foot_trajectory_global.RFoot_euler(i);
        }
    }
}


void Robot_Control::Impedance_update()
{
    for(int i=0; i<3;i++)
    {
        if(_cnt == 0)
        {
            _Impedance_T_R_prev2(i) = _T_RFoot_global[5].translation()(i);
            _Impedance_T_L_prev2(i) = _T_LFoot_global[5].translation()(i);

            _Impedance_T_R_prev1(i) = _T_RFoot_global[5].translation()(i);
            _Impedance_T_L_prev1(i) = _T_LFoot_global[5].translation()(i);
        }
        else
        {
            _Impedance_T_R_prev2(i) = _Impedance_T_R_prev1(i);
            _Impedance_T_L_prev2(i) = _Impedance_T_L_prev1(i);

            _Impedance_T_R_prev1(i) = _Impedance_T_R_current(i);
            _Impedance_T_L_prev1(i) = _Impedance_T_L_current(i);
        }
        _Impedance_T_R_current(i) = _T_RFoot_global[5].translation()(i);
        _Impedance_T_L_current(i) = _T_LFoot_global[5].translation()(i);


        if(_cnt == 0)
        {
            _Impedance_T_R_prev2(i+3) = _T_RFoot_global_euler(i);
            _Impedance_T_L_prev2(i+3) = _T_LFoot_global_euler(i);


            _Impedance_T_R_prev1(i+3) =  _T_RFoot_global_euler(i);
            _Impedance_T_L_prev1(i+3) =  _T_LFoot_global_euler(i);
        }
        else
        {
            _Impedance_T_R_prev2(i+3) = _Impedance_T_R_prev1(i+3);
            _Impedance_T_L_prev2(i+3) = _Impedance_T_L_prev1(i+3);


            _Impedance_T_R_prev1(i+3) = _Impedance_T_R_current(i+3);
            _Impedance_T_L_prev1(i+3) = _Impedance_T_L_current(i+3);
        }
        _Impedance_T_R_current(i+3) = _T_RFoot_global_euler(i);
        _Impedance_T_L_current(i+3) = _T_LFoot_global_euler(i);
    }


    for(int i=0; i<3;i++)
    {
        if(_cnt == 0)
        {
            _Impedance_T_R_desired_prev2(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_T_R_desired_prev1(i) = Foot_trajectory_global.RFoot.translation()(i);
            _Impedance_T_L_desired_prev2(i) = Foot_trajectory_global.LFoot.translation()(i);
            _Impedance_T_L_desired_prev1(i) = Foot_trajectory_global.LFoot.translation()(i);
        }
        else
        {
            _Impedance_T_L_desired_prev2(i) = _Impedance_T_L_desired_prev1(i);
            _Impedance_T_R_desired_prev2(i) = _Impedance_T_R_desired_prev1(i);

            _Impedance_T_R_desired_prev1(i) = _Impedance_T_R_desired_current(i);
            _Impedance_T_L_desired_prev1(i) = _Impedance_T_L_desired_current(i);
        }
        _Impedance_T_R_desired_current(i) = Foot_trajectory_global.RFoot.translation()(i);
        _Impedance_T_L_desired_current(i) = Foot_trajectory_global.LFoot.translation()(i);


        if(_cnt == 0)
        {
            _Impedance_T_R_desired_prev2(i+3) =  Foot_trajectory_global.RFoot_euler(i);
            _Impedance_T_L_desired_prev2(i+3) =  Foot_trajectory_global.LFoot_euler(i);

            _Impedance_T_R_desired_prev1(i+3) =  Foot_trajectory_global.RFoot_euler(i);
            _Impedance_T_L_desired_prev1(i+3) =  Foot_trajectory_global.LFoot_euler(i);
        }
        else
        {
            _Impedance_T_L_desired_prev2(i+3) = _Impedance_T_L_desired_prev1(i+3);
            _Impedance_T_R_desired_prev2(i+3) = _Impedance_T_R_desired_prev1(i+3);

            _Impedance_T_R_desired_prev1(i+3) = _Impedance_T_R_desired_current(i+3);
            _Impedance_T_L_desired_prev1(i+3) = _Impedance_T_L_desired_current(i+3);
        }
        _Impedance_T_R_desired_current(i+3) = Foot_trajectory_global.RFoot_euler(i);
        _Impedance_T_L_desired_current(i+3) = Foot_trajectory_global.LFoot_euler(i);
    }

    for(int i =0; i<6; i++)
    {
        if(_cnt == 0)
        {
            R_position_dot.setZero();
            L_position_dot.setZero();
            R_position_dot_desired.setZero();
            L_position_dot_desired.setZero();
        }
        else
        {
            R_position_dot(i) = (_Impedance_T_R_current(i)-_Impedance_T_R_prev1(i))*Hz;
            L_position_dot(i) = (_Impedance_T_L_current(i)-_Impedance_T_L_prev1(i))*Hz;
            R_position_dot_desired(i) = (_Impedance_T_R_desired_current(i)-_Impedance_T_R_desired_prev1(i))*Hz;
            L_position_dot_desired(i) = (_Impedance_T_L_desired_current(i)-_Impedance_T_L_desired_prev1(i))*Hz;
        }
    }

}


void Robot_Control::Impedance_control()
{
    ////////////Impedance ����//////////////////

    if(_cnt == 0)
        _Impedance_flag = false;

    double offset_time = 0.4*Hz;
    double FT_Threshold = 50.0;

    /////////Impedance ������ �� ���//////////////
    Vector6D FT;
    if(_foot_step(_step_number,6) == 1)
        FT = _R_FT_global;
    else
        FT = _L_FT_global;

    //////////���� ������� Double support �ð��� �Ǹ� Impedance flag ����///////////////////



    ////////////////landing ��� 0.3�� ������ �����ϰ� ���////////////////////////////
    if((_Impedance_flag == false) && (_cnt > _T_Start+_T_Total-_T_Double2-_T_rest_init-offset_time) && (_cnt < _T_Start+_T_Total))
    {
        if(FT(2) > FT_Threshold || FT(3) > 10.0 || FT(4) > 10.0)
        {
            _Impedance_flag = true;
            initial_state.time = _cnt;
            initial_state.FT = FT;
            initial_state.LFoot_current = _T_LFoot_global[5].translation();
            initial_state.RFoot_current = _T_RFoot_global[5].translation();
            initial_state.LFoot_linear = _T_LFoot_global[5].linear();
            initial_state.RFoot_linear = _T_RFoot_global[5].linear();
            initial_state.LFoot_euler = _T_LFoot_global_euler;
            initial_state.RFoot_euler = _T_RFoot_global_euler;
            cout << "_cnt" << _cnt << endl;
            cout << "_step_number" << _step_number << endl;
            cout << "Lfoot" << initial_state.LFoot_current << endl;
            cout << "Rfoot" << initial_state.RFoot_current << endl;
            cout << "Impedance start"<<endl;

            Xe_LFoot.resize(2,1); Xe_LFoot.setZero();
            Xe_RFoot.resize(2,1); Xe_RFoot.setZero();
            Xe_LFoot_Roll.resize(2,1); Xe_LFoot_Roll.setZero();
            Xe_RFoot_Roll.resize(2,1); Xe_RFoot_Roll.setZero();
            Xe_LFoot_Pitch.resize(2,1); Xe_LFoot_Pitch.setZero();
            Xe_RFoot_Pitch.resize(2,1); Xe_RFoot_Pitch.setZero();


        }
        if(_cnt == _T_Start+_T_Total-_T_rest_init-_T_Double2-_T_Imp)
        {
            cout << "delay_landing" << endl;
            _Impedance_flag = true;
            initial_state.time = _cnt;
            initial_state.FT = FT;
            initial_state.LFoot_current = _T_LFoot_global[5].translation();
            initial_state.RFoot_current = _T_RFoot_global[5].translation();
            initial_state.LFoot_linear = _T_LFoot_global[5].linear();
            initial_state.RFoot_linear = _T_RFoot_global[5].linear();
            initial_state.LFoot_euler = _T_LFoot_global_euler;
            initial_state.RFoot_euler = _T_RFoot_global_euler;


            Xe_LFoot.resize(2,1); Xe_LFoot.setZero();
            Xe_RFoot.resize(2,1); Xe_RFoot.setZero();
            Xe_LFoot_Roll.resize(2,1); Xe_LFoot_Roll.setZero();
            Xe_RFoot_Roll.resize(2,1); Xe_RFoot_Roll.setZero();
            Xe_LFoot_Pitch.resize(2,1); Xe_LFoot_Pitch.setZero();
            Xe_RFoot_Pitch.resize(2,1); Xe_RFoot_Pitch.setZero();
        }
    }

    if(_cnt == _T_Start)
    {
        _Impedance_flag = false;
        cout <<  _cnt << " Impedance end" << endl;
    }
    else if ((_step_number == _step_total_number-1) && (_cnt == _T_Start+_T_Total))
    {
        _Impedance_flag = false;
        cout << _cnt << "last Impedance end" << endl;
    }


   // if(_cnt > 5)
   //     _Impedance_flag = true;

   // _Impedance_flag = true; // test
    ///////////Impedance control ���� /////////////////////////
    if(_Impedance_flag == true)
    {
        double Mg = 48.3628*9.81; // �κ� ����
        //////desired force moment ���///
        RFT_desired.setZero();
        LFT_desired.setZero();
        if(_cnt < _T_Start+_T_Total)
        {
            if(_foot_step(_step_number,6) == 1)
            {
                if(_cnt < _T_Start+_T_Total-_T_rest_init)
                {
                    //RFT_desired(2) = (Mg*0.5-initial_state.FT(2))*(_cnt-initial_state.time)/(_T_Start+_T_Total-_T_Double2-initial_state.time)+initial_state.FT(2);
                    RFT_desired(2) = (Mg*0.5-initial_state.FT(2))*(_cnt-initial_state.time)/(_T_Start+_T_Total-_T_rest_init-initial_state.time)+initial_state.FT(2);
                    LFT_desired(2) = Mg-RFT_desired(2);
                }
                else
                {
                    RFT_desired(2) = Mg*0.5;
                    LFT_desired(2) = Mg-RFT_desired(2);
                }
            }
            else
            {
                if(_cnt < _T_Start+_T_Total-_T_rest_init)
                {
                    //LFT_desired(2) = (Mg*0.5-initial_state.FT(2))*(_cnt-initial_state.time)/(_T_Start+_T_Total-_T_Double2-initial_state.time)+initial_state.FT(2);
                    LFT_desired(2) = (Mg*0.5-initial_state.FT(2))*(_cnt-initial_state.time)/(_T_Start+_T_Total-_T_rest_init-initial_state.time)+initial_state.FT(2);
                    RFT_desired(2) = Mg-LFT_desired(2);
                }
                else
                {
                    LFT_desired(2) = Mg*0.5;
                    RFT_desired(2) = Mg-LFT_desired(2);
                }
            }
        }
        else
        {
            RFT_desired(2) = 0.5*Mg;
            LFT_desired(2) = 0.5*Mg;
        }

      //  RFT_desired.setZero();
      //  LFT_desired.setZero();

        double m_imp = 20.0;
        double d_imp = 6000.0;
        double k_imp = 0.1;

        double m_ori_imp = 5.0;
        double d_ori_imp = 100.0;
        double k_ori_imp = 0.1;

        double del_t = 1.0/Hz;

        Vector6D LF_error;
        LF_error = (LFT_desired-_L_FT_global);
        Vector6D LX_error;
        LX_error = _Impedance_T_L_desired_prev1-_Impedance_T_L_prev1;
        Vector6D LX_error_1;
        LX_error_1 =  (_Impedance_T_L_desired_prev1-_Impedance_T_L_desired_prev2)/del_t-(_Impedance_T_L_prev1-_Impedance_T_L_prev2)/del_t;

        Vector6D RF_error;
        RF_error = (RFT_desired-_R_FT_global);
        Vector6D RX_error;
        RX_error = _Impedance_T_R_desired_prev1-_Impedance_T_R_prev1;
        Vector6D RX_error_1;
        RX_error_1 =  (_Impedance_T_R_desired_prev1-_Impedance_T_R_desired_prev2)/del_t-(_Impedance_T_R_prev1-_Impedance_T_R_prev2)/del_t;



        Imp_FT_debug(0) = RFT_desired(2);
        Imp_FT_debug(1) = RFT_desired(3);
        Imp_FT_debug(2) = RFT_desired(4);
        Imp_FT_debug(3) = _R_FT_global(2);
        Imp_FT_debug(4) = _R_FT_global(3);
        Imp_FT_debug(5) = _R_FT_global(4);

        Imp_FT_debug(6) = LFT_desired(2);
        Imp_FT_debug(7) = LFT_desired(3);
        Imp_FT_debug(8) = LFT_desired(4);
        Imp_FT_debug(9) = _L_FT_global(2);
        Imp_FT_debug(10) = _L_FT_global(3);
        Imp_FT_debug(11) = _L_FT_global(4);



        MatrixXD Ad;
        Ad.resize(2,2); Ad.setZero();
        MatrixXD Bd;
        Bd.resize(2,1); Bd.setZero();

        Ad(0,0) = 1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(-d_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)+d_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)+exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)+exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp))*(1.0/2.0);
        Ad(0,1) = -(m_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-m_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp))*1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp);
        Ad(1,0) = (k_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-k_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp))*1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp);
        Ad(1,1) = 1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(d_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-d_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)+exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)+exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp))*(1.0/2.0);

        Bd(0,0) = -((m_imp*(1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(d_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-d_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)+exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)+exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp))*(1.0/2.0)-1.0))/k_imp-(d_imp*(m_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-m_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp))*1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp))/k_imp)/(m_imp);
        Bd(1,0) = -((m_imp*exp((d_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp)-m_imp*exp((d_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_imp*m_imp*-4.0+d_imp*d_imp)*(1.0/2.0))/m_imp))*1.0/sqrt(k_imp*m_imp*-4.0+d_imp*d_imp))/(m_imp);


        MatrixXD Ad_ori;
        Ad_ori.resize(2,2); Ad_ori.setZero();
        MatrixXD Bd_ori;
        Bd_ori.resize(2,1); Bd_ori.setZero();

        Ad_ori(0,0) = 1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(-d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)+d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp))*(1.0/2.0);
        Ad_ori(0,1) = -(m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp))*1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp);
        Ad_ori(1,0) = (k_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-k_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp))*1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp);
        Ad_ori(1,1) = 1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp))*(1.0/2.0);

        Bd_ori(0,0) = -((m_ori_imp*(1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-d_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)+exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp))*(1.0/2.0)-1.0))/k_ori_imp-(d_ori_imp*(m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp))*1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp))/k_ori_imp)/(m_ori_imp);
        Bd_ori(1,0) = -((m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)-del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp)-m_ori_imp*exp((d_ori_imp*del_t*(-1.0/2.0)+del_t*sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp)*(1.0/2.0))/m_ori_imp))*1.0/sqrt(k_ori_imp*m_ori_imp*-4.0+d_ori_imp*d_ori_imp))/(m_ori_imp);


        MatrixXD Xe_plus_LFoot;
        Xe_plus_LFoot.resize(2,1);
        MatrixXD Xe_plus_RFoot;
        Xe_plus_RFoot.resize(2,1);
        MatrixXD Xe_plus_LFoot_Roll;
        Xe_plus_LFoot_Roll.resize(2,1);
        MatrixXD Xe_plus_RFoot_Roll;
        Xe_plus_RFoot_Roll.resize(2,1);
        MatrixXD Xe_plus_LFoot_Pitch;
        Xe_plus_LFoot_Pitch.resize(2,1);
        MatrixXD Xe_plus_RFoot_Pitch;
        Xe_plus_RFoot_Pitch.resize(2,1);


        Xe_plus_LFoot = Ad*Xe_LFoot+Bd*LF_error(2);
        Xe_plus_RFoot = Ad*Xe_RFoot+Bd*RF_error(2);

        Xe_plus_LFoot_Roll = Ad_ori*Xe_LFoot_Roll+Bd_ori*LF_error(3);
        Xe_plus_RFoot_Roll = Ad_ori*Xe_RFoot_Roll+Bd_ori*RF_error(3);

        Xe_plus_LFoot_Pitch = Ad_ori*Xe_LFoot_Pitch+Bd_ori*LF_error(4);
        Xe_plus_RFoot_Pitch = Ad_ori*Xe_RFoot_Pitch+Bd_ori*RF_error(4);


        Xe_LFoot = Xe_plus_LFoot;
        Xe_RFoot = Xe_plus_RFoot;


        Xe_LFoot_Roll = Xe_plus_LFoot_Roll;
        Xe_RFoot_Roll = Xe_plus_RFoot_Roll;

        Xe_LFoot_Pitch = Xe_plus_LFoot_Pitch;
        Xe_RFoot_Pitch = Xe_plus_RFoot_Pitch;





        ///////������ġ//////

        if(_foot_step(_step_number,6) == 0) // �޹� ����
        {

            Foot_trajectory_global.LFoot.translation()(2) = Foot_trajectory_global.LFoot.translation()(2)-Xe_LFoot(0,0);
            //Foot_trajectory_global.LFoot_euler(0) = Foot_trajectory_global.LFoot_euler(0)-Xe_LFoot_Roll(0,0);
            //Foot_trajectory_global.LFoot_euler(1) = Foot_trajectory_global.LFoot_euler(1)-Xe_LFoot_Pitch(0,0);

            Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));
            Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));
        }
        else if(_foot_step(_step_number,6) == 1)
        {
            Foot_trajectory_global.RFoot.translation()(2) = Foot_trajectory_global.RFoot.translation()(2)-Xe_RFoot(0,0);
          //  Foot_trajectory_global.RFoot_euler(0) = Foot_trajectory_global.RFoot_euler(0)-Xe_RFoot_Roll(0,0);
          //  Foot_trajectory_global.RFoot_euler(1) = Foot_trajectory_global.RFoot_euler(1)-Xe_RFoot_Pitch(0,0);

           Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));
            Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));
        }


         if((Foot_trajectory_global.LFoot.translation()(2)-_init_info._XL_global_init.translation()(2)) > 0.05)
             Foot_trajectory_global.LFoot.translation()(2) = _init_info._XL_global_init.translation()(2) +0.05;

         if((Foot_trajectory_global.LFoot.translation()(2)-_init_info._XL_global_init.translation()(2)) < -0.05)
             Foot_trajectory_global.LFoot.translation()(2) = _init_info._XL_global_init.translation()(2) -0.05;


         if((Foot_trajectory_global.RFoot.translation()(2)-_init_info._XR_global_init.translation()(2)) > 0.05)
             Foot_trajectory_global.RFoot.translation()(2) = _init_info._XR_global_init.translation()(2) +0.05;

         if((Foot_trajectory_global.RFoot.translation()(2)-_init_info._XR_global_init.translation()(2)) < -0.05)
             Foot_trajectory_global.RFoot.translation()(2) = _init_info._XR_global_init.translation()(2) -0.05;


       }




}

