#ifndef __UPPER_PARAMETER__
#define __UPPER_PARAMETER__

#include "DefineVar.h"

class UpperPara
{
public:
	FILE* fp1;

	int		_Hz;
    int _gap;
    int _gap2;


	struct Initinfo{
		int			t;
		VectorXD	q;
		HTransform  T_Rarm;
		HTransform  T_Larm;
		Vector3D    Rarm_euler;
		Vector3D    Larm_euler;
	};
	struct Larm{
		Vector3D EulerAngle;
		HTransform T;
		MatrixXD J;
		MatrixXD J_inv;
		MatrixXD J_inv_sin;
		VectorXD q;
	};
	struct Rarm{
		Vector3D EulerAngle;
		HTransform T;
		MatrixXD J;
		MatrixXD J_inv;
		//MatrixXD J_inv_sin;
		VectorXD q;
	};


public:
	Initinfo	_init_info;
	
public:
	void set_user_value();
	void Upperpara_initialize();

public:
	VectorXD	_target_q;
	VectorXD	_desired_q;
	int			_cnt;
	double		_duration;
	VectorXD	_q;
	
	////////////// CLIK Algorithm ///////////////

public:
	int			_target_num;
	int			_target_state; 
	
	int			_pre_time;
	int			_current_task_time;
	VectorXD	_pre_target;
	VectorXD        _current_target;

	MatrixXD	_target_global;
	VectorXD	_target_local;

	bool		_rel;
	bool		_singularity;
	double		_Clik_gain;
	double		_Singularity_gain;
	int			_task_time;

	Vector6D	_target_x;
	Vector6D	_cubic_x;
	Vector3D	_pre_x;
	Vector6D	_xdot_d;
	Vector6D	_error;
	VectorXD	_qdot_d;

	bool		_arm_type; // 0: Right arm, 1: Left Arm

	Rarm		_Rarm;
	Larm		_Larm;

	Vector3D	_rphi;
	Vector3D	_lphi;

	double		_rmax;
	double  	_w0;
    double          _lambda;
    double          _w;

    //////sensors /////////
    Vector3D _Gyro_data;
    Matrix3D _gyroMatrix;
    bool  _gyro_flag; // for on-off gyro sensor feedback


};
#endif
