#include "Walking_Parameter.h"

void WalkingPara::set_user_value()
{
    Hz = 200.0;
    _ho = 0.05;
    _T_Double1 = 0.1*Hz;
    _T_Double2 = 0.1*Hz;

    _T_Total = 3.0*Hz;

    _T_temp = 3.0*Hz;
    _T_Imp = 0.0*Hz;

    _T_rest_init = 1.0*Hz;
    _T_rest_last = 1.0*Hz;

    _gyro_frame_flag = false;
    _com_control_flag = false;
    _landing_flag = false;

    _ZMP_left_offset = 0.015;
    _ZMP_right_offset = -0.01;
}

void WalkingPara::WALKINGpara_initialize()
{

    for(int i=0; i<FILE_CNT;i++)
    {
        if(file[i].is_open())
            file[i].close();

        file[i].open(FILE_NAMES[i].c_str());
    }


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
