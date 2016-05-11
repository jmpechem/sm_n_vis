#include "Walking_Parameter.h"

void WalkingPara::set_user_value()
{
    Hz = 400.0;
    _ho = 0.05;
    _T_Double1 = 0.2*Hz;
    _T_Double2 = 0.2*Hz;
    _T_Total = 1.5*Hz;
    _T_temp = 3.0*Hz;
    _T_Imp = 0.1*Hz;

    _gyro_frame_flag = true;
    _com_control_flag = true;
    _landing_flag = true;


    _ZMP_left_offset = 0.0;
    _ZMP_right_offset = 0.0;
}

void WalkingPara::WALKINGpara_initialize()
{
    fp1 = fopen("../acquisition/Desired_ZMP.txt","w");
    fp2 = fopen("../acquisition/Desired_COM.txt","w");
    fp3 = fopen("../acquisition/Desired_q.txt","w");
    fp4 = fopen("../acquisition/Real_q.txt","w");
    fp5 = fopen("../acquisition/Desired_Foot.txt","w");
    fp6 = fopen("../acquisition/Desired_Footori.txt","w");
    fp7 = fopen("../acquisition/Contact_Force.txt","w");
    fp8 = fopen("../acquisition/Impedance.txt","w");
    fp9 = fopen("../acquisition/Current_Foot2.txt","w");
    fp10 = fopen("../acquisition/Htransform.txt","w");
    fp11 = fopen("../acquisition/Real_Foot.txt","w");
    fp12 = fopen("../acquisition/Real_Foot_ori.txt","w");
    fp13 = fopen("../acquisition/COM-FOOTsupport.txt","w");

    _cnt = 0;
    _step_number = 0;
    _step_total_number = 0;
    ////////////Robot low data///////////
    _q.resize(28);
    _q.setZero();
    _desired_q.resize(28);
    _desired_q.setZero();
    _L_Ft.setZero();
    _R_Ft.setZero();
    _Gyro_Base.setZero();
    _Gyro_LFoot.setZero();
    _Gyro_RFoot.setZero();
    _L_FT_lowpass.setZero();
    _R_FT_lowpass.setZero();
    _L_FT_global.setZero();
    _R_FT_global.setZero();
    ////////////init joint angle////////
    _init_q.resize(28);
    _init_q.setZero();
    ///////////////////////////////////
    _Step_Planning_flag = false;


    /////Kinematic////////////////////
    _COM_Rarm_support.setZero();
    _COM_Larm_support.setZero();
    _COM_Rfoot_support.setZero();
    _COM_Lfoot_support.setZero();
    _COM_Base_support.setZero();
    _COM_real_support.setZero();

    _ZMP_real.setZero();
    _ZMP_desired.setZero();;

    _COM_update_flag = false;


    for (int i=0; i<6; i++)
    {
        _J_RFoot_global[i].resize(6,i+1);
        _J_LFoot_global[i].resize(6,i+1);
    }
}
