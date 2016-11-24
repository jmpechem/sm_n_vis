#ifndef __WALKING_PARAMETER__
#define __WALKING_PARAMETER__

#include "DefineVar.h"
#include <ros/ros.h>
#include <fstream>

const int FILE_CNT = 15;

const string FILE_NAMES[FILE_CNT] = {
    "/home/pen/DAQ/Walking/Desired_ZMP.txt",
    "/home/pen/DAQ/Walking/Desired_COM.txt",
    "/home/pen/DAQ/Walking/Desired_q.txt",
    "/home/pen/DAQ/Walking/Real_q.txt",
    "/home/pen/DAQ/Walking/Desired_Foot.txt",
    "/home/pen/DAQ/Walking/Desired_Footori.txt",
    "/home/pen/DAQ/Walking/Contact_Force.txt",
    "/home/pen/DAQ/Walking/Impedance.txt",
    "/home/pen/DAQ/Walking/Current_Foot2.txt",
    "/home/pen/DAQ/Walking/Htransform.txt",
    "/home/pen/DAQ/Walking/Real_Foot.txt",
    "/home/pen/DAQ/Walking/Real_Foot_ori.txt",
    "/home/pen/DAQ/Walking/COM-FOOTsupport.txt",
};


class WalkingPara
{
public:
    fstream file[FILE_CNT];

public:

	double COM_pd;
	int Hz;
	Vector3D COM_prev;
    typedef struct {
        HTransform	_XL_global_init;
        HTransform	_XR_global_init;
        HTransform	_trunk_global_init;
        Vector3D	_XL_global_euler_init;
        Vector3D	_XR_global_euler_init;
        Vector3D	_trunk_global_euler_init;
        Vector3D	_COM_global_init;
        HTransform	_XL_support_init;
        HTransform	_XR_support_init;
        HTransform	_trunk_support_init;
        Vector3D	_XL_support_euler_init;
        Vector3D	_XR_support_euler_init;
        Vector3D	_trunk_support_euler_init;
        Vector3D	_COM_support_init;
        VectorXD	q;
    } Initinfo;

    typedef struct {
        HTransform	LFoot;
        HTransform	RFoot;
        Vector3D	LFoot_euler;
        Vector3D	RFoot_euler;
        Vector6D    LFoot_dot;
        Vector6D    RFoot_dot;
    } Desired_Foot;

    typedef struct {
        Vector3D _COM;
        HTransform _Trunk;
        Vector3D _Trunk_euler;
    } init_COM;


public:
    VectorXD                Imp_FT_debug;
    bool                    _global_walking_flag;
    VectorXD                _Joint_Offset_Angle;
    Vector6D                _Impedance_output_LFoot;
    Vector6D                _Impedance_output_RFoot;

    Vector3D                _Target_data;
    Vector3D                _scan_data;
	Vector3D				_Base_COM;
	Vector3D				_COM_R[6];
	Vector3D				_COM_L[6];
	Vector3D				_COM_Rarm[7];
	Vector3D				_COM_Larm[7];

	Vector3D				_m_R[6];
	Vector3D				_m_L[6];
	Vector3D				_m_Rarm[7];
	Vector3D				_m_Larm[7];
	Vector3D				_m_trunk[3];
	Vector3D				_m_pelvis;
	Vector3D				_m_torso;
	Vector3D				_m_waist;


	Matrix3D _inertia_hat[28];
	Vector3D _Momentum_Mass_column[28]; // total_dof
	Vector3D _Momentum_Inertia_column[28]; // total_dof

	Vector3D _Momentum_COM_hat[28];
	VectorXD mass_hat;


	VectorXD				Mass;
	Matrix3D		_Inertia_link[29];
	Vector3D p_ref;
	Vector3D L_ref;

	Vector6D Desired_LFoot_dot;
	Vector6D Desired_RFoot_dot;

	MatrixXD _J_global[28];
	MatrixXD _J_COM_global[28];

	MatrixXD _Momentum_Mass;
	MatrixXD _Momentum_Inertia_global;
	MatrixXD _Momentum_Inertia_com;


	Vector3D ZMP_real_val;
	///////user setting/////////////
	bool _landing_flag;
	bool _gyro_frame_flag;
	bool _com_control_flag;


	Vector3D _Base_position;
	Vector3D _RFoot_position;
	Vector3D _LFoot_position;

	int _cnt;
	int _step_number;
	int _step_total_number;
	//////////////œÃ°£//////////////////////
    int _T_Double1; // ¹«°ÔÁßœÉ ¹ß·Î ¿Å±âŽÂ œÃ°£
    int _T_Total;
    int _T_Double2; //¹ß¿¡Œ­ ÁßœÉÀž·Î ¿ÀŽÂ œÃ°£
    int _T_temp;
    int _T_Start;
    int _T_Start_real;
    int _T_Last;
    int _T_Imp;
    int _T_rest_init;
    int _T_rest_last;
	////////// FootPlanning/////////////////
    MatrixXD			_foot_step;
    bool				_Step_Planning_flag;
    MatrixXD				_foot_step_localframe;
    MatrixXD				_foot_step_localframe_offset;
    //////////Robot low data///////////
    VectorXD _q;
    VectorXD _desired_q;
    Vector6D _L_Ft;
    Vector6D _R_Ft;
    Vector3D _Gyro_Base;
    Vector3D _Gyro_LFoot;
    Vector3D _Gyro_RFoot;
    Vector6D _L_FT_lowpass;
    Vector6D _R_FT_lowpass;
    Vector6D _L_FT_global;
    Vector6D _R_FT_global;
    ///////////Kinematic/////////////////////
    MatrixXD			_J_RFoot_global[6];
    MatrixXD			_J_LFoot_global[6];

    HTransform			_T_RFoot_global[6];
    HTransform			_T_LFoot_global[6];
    HTransform			_T_RArm_global[7];
    HTransform			_T_LArm_global[7];
    HTransform			_T_Waist_global[2];
    HTransform			_T_Trunk_global;
    HTransform			_T_RFoot_support[6];
    HTransform			_T_LFoot_support[6];
    HTransform			_T_RArm_support[7];
    HTransform			_T_LArm_support[7];
    HTransform			_T_Trunk_support;

    Vector3D			_T_LFoot_global_euler;
    Vector3D			_T_RFoot_global_euler;
    Vector3D			_T_LFoot_support_euler;
    Vector3D			_T_RFoot_support_euler;
    Vector3D			_T_Trunk_support_euler;

    Vector3D			_COM_Rarm_support;
    Vector3D			_COM_Larm_support;
    Vector3D			_COM_Rfoot_support;
    Vector3D			_COM_Lfoot_support;
    Vector3D			_COM_Base_support;
    Vector3D			_COM_real_support;
    Vector3D			_COM_real_global;

    Initinfo			_init_info;
    ///////////////////////////////////////////
    ///////////////////ZMP °ü·Ã º¯Œö///////////
    MatrixXD			_Ref_ZMP;
    double				_ZMP_left_offset;
    double				_ZMP_right_offset;

    Vector3D			_ZMP_real;
    Vector3D			_ZMP_desired;

    init_COM			_init_COM;
    bool				_COM_update_flag;
    Vector3D			_COM_desired;

    VectorXD _init_q;

    Desired_Foot		_init_Foot_trajectory;
    Desired_Foot		Foot_trajectory;
    Desired_Foot		Foot_trajectory_global;
    double				_ho;

    HTransform			Trunk_trajectory;
    HTransform			Trunk_trajectory_global;

public:
    void WALKINGpara_initialize();
    void set_user_value();
};
#endif
