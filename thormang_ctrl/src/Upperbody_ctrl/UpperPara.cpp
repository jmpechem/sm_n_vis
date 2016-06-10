#include "UpperPara.h"

void UpperPara::set_user_value()
{
    _Hz = 333;
}

void UpperPara::Upperpara_initialize()
{
	_cnt = 0;

    _q.resize(32);
	_q.setZero();
    _desired_q.resize(32);
	_desired_q.setZero();
    _target_q.resize(32);
	_target_q.setZero();

	_duration = 0.0;

	_target_num = 0;
	_target_state =0; 

	_pre_time = 0;
	_task_time = 0;
	_current_task_time = 0;

	_pre_target.resize(8);
	_pre_target.setZero();
	_current_target.resize(8);
	_current_target.setZero();
	_target_local.resize(8);
	_target_local.setZero();

	_rel = false;
	_singularity = false;
	_Clik_gain = 100.0;
	_Singularity_gain = 100.0;
	
	_task_time =0;

	_qdot_d.resize(7);
	_qdot_d.setZero();
	_error.setZero();
	_xdot_d.setZero();

	_Rarm.q.resize(7);
	_Rarm.q.setZero();
	_Larm.q.resize(7);
	_Larm.q.setZero();

	_Rarm.J_inv.resize(7,6);
	_Larm.J_inv.resize(7,6);

	_Rarm.EulerAngle.setZero();
	_Rarm.J.setZero();
	_Rarm.J_inv.setZero();

	_rmax = pi/4;
	

}
