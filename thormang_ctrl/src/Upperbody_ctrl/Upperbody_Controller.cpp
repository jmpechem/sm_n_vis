#include "Upperbody_Controller.h"

UpperCtrl::UpperCtrl(){};
UpperCtrl::~UpperCtrl(){};
#define  JOINT_NUM 28

template<typename _Matrix_Type_>
_Matrix_Type_ pinv(const _Matrix_Type_ &a, double epsilon =std::numeric_limits<double>::epsilon())
{
	JacobiSVD< _Matrix_Type_ > svd(a ,ComputeThinU | ComputeThinV);
	double tolerance = epsilon * max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}
void UpperCtrl::IK_compute(VectorXD& output)
{
	Get_IK_Value();
	
	if (_target_state < _target_num && _cnt == _pre_time+_current_task_time) {
		SET_IK_Initialize();
		SET_IK_Local_Target();
		output.resize(28);
		output = _q;
	}
	if (_cnt <= _task_time)
		IK_solver(output);
		
	if (_cnt == _task_time){
		cout << "" << endl;
		cout << "Final Value"  << endl;
		cout << "T_Rarm: " << _Rarm.T.translation()(0) <<" " << _Rarm.T.translation()(1) <<" " << _Rarm.T.translation()(2) << endl << "T_Larm: " << _Larm.T.translation()(0) <<" " << _Larm.T.translation()(1) <<" " << _Larm.T.translation()(2) << endl;
		cout << "Euler_Rarm:" << _Rarm.EulerAngle(0) <<" " << _Rarm.EulerAngle(1) <<" " << _Rarm.EulerAngle(2) << endl << "Euler_Larm:" << _Larm.EulerAngle(0) <<" " << _Larm.EulerAngle(1) <<" " << _Larm.EulerAngle(2) << endl; ; 
	}
	_cnt++;

}
void UpperCtrl::SET_IK_Initialize()
{
	_init_info.t = _cnt;
	_init_info.q = _q;
	_init_info.T_Rarm = _Rarm.T;
	_init_info.T_Larm = _Larm.T;
	_init_info.Larm_euler = _Larm.EulerAngle;
	_init_info.Rarm_euler = _Rarm.EulerAngle;

	cout << "" << endl;
	cout << "Initial Value#" << _target_state+1 << endl;
	cout << "T_Rarm: " << _Rarm.T.translation()(0) <<" " << _Rarm.T.translation()(1) <<" " << _Rarm.T.translation()(2) << endl << "T_Larm: " << _Larm.T.translation()(0) <<" " << _Larm.T.translation()(1) <<" " << _Larm.T.translation()(2) << endl;
	cout << "Euler_Rarm:" << _Rarm.EulerAngle(0) <<" " << _Rarm.EulerAngle(1) <<" " << _Rarm.EulerAngle(2) << endl << "Euler_Larm:" << _Larm.EulerAngle(0) <<" " << _Larm.EulerAngle(1) <<" " << _Larm.EulerAngle(2) << endl; 
}
void UpperCtrl::IK_solver(VectorXD& output)
{
	if (_arm_type){
		MatrixXD J_Temp;
                J_Temp = _Rarm.J* _Rarm.J.transpose();
		
                _w = sqrt(J_Temp.determinant());

		if (_singularity && _w <= _w0){
			MatrixXD J_T = _Rarm.J.transpose();
			MatrixXD I;
			I.setIdentity(6,6);
			double a = _lambda*pow(1-_w/_w0,2);
			MatrixXD J_Damped;
			J_Damped = _Rarm.J*_Rarm.J.transpose()+a*I;
			J_Damped = J_Damped.inverse();

			cout << "Singularity Region: " << _w << endl;
			_Rarm.J_inv = _Rarm.J.transpose()*J_Damped;			
		}
		else {
			_Rarm.J_inv = pinv(_Rarm.J);
		}
		
		if (_cnt >= _init_info.t && _cnt < _init_info.t + _current_task_time + 1.0*_Hz) 
			IK_solver_basic(output);
	}
	else{
		MatrixXD J_Temp;
                J_Temp = _Larm.J* _Larm.J.transpose();		
                _w = sqrt(J_Temp.determinant());

		if (_singularity && _w <= _w0){
			MatrixXD J_T = _Larm.J.transpose();
			MatrixXD I;
			I.setIdentity(6,6);
			double a = _lambda*pow(1-_w/_w0,2);
			MatrixXD J_Damped;
			J_Damped = _Rarm.J*_Larm.J.transpose()+a*I;
			J_Damped = J_Damped.inverse();

			cout << "Singularity Region: " << _w << endl;
			_Larm.J_inv = _Larm.J.transpose()*J_Damped;			
		}
		else {
			_Larm.J_inv = pinv(_Larm.J);
		}
		if (_cnt >= _init_info.t && _cnt < _init_info.t + _current_task_time + 1.0*_Hz) 
		        IK_solver_basic(output);
	}
}
void UpperCtrl::IK_solver_basic(VectorXD& output)
{
	if (_arm_type) {				
		if (_cnt>= _init_info.t && _cnt <= _init_info.t + _current_task_time) {
			for (int i=0; i<3; i++) {
				_cubic_x(i) = Cubic(_cnt, _init_info.t, _init_info.t+ _current_task_time, _init_info.T_Rarm.translation()(i), 0.0, _target_x(i), 0.0);
				_cubic_x(i+3) = Cubic(_cnt, _init_info.t, _init_info.t+ _current_task_time, 0.0, 0.0, _target_x(i+3), 0.0);
				}
			GetPhi(_Rarm.T, _init_info.T_Rarm, _cubic_x, _rphi);

			if (_cnt == _init_info.t)
				_pre_x = _Rarm.T.translation();

			for (int i=0; i<3; i++){
				_xdot_d(i) = (_cubic_x(i)- _pre_x(i))*_Hz;
				_xdot_d(i+3) = -_rphi(i);
				_pre_x(i) = _cubic_x(i);
				}
			for (int i=0; i<3; i++){
				_error(i) = _cubic_x(i) - _Rarm.T.translation()(i);
				_error(i+3) = -_rphi(i);
				}
			}
		else if (_cnt > _init_info.t+_current_task_time) {
			_cubic_x = _target_x;
			GetPhi(_Rarm.T, _init_info.T_Rarm, _cubic_x, _rphi);
			_xdot_d.setZero();
			for (int i=0; i<3; i++){
				_error(i) = _cubic_x(i) - _Rarm.T.translation()(i);
				_error(i+3) = -_rphi(i);
			}
		}
		else {
			_Rarm.J_inv.setZero();
		}
		_qdot_d = _Rarm.J_inv * (_xdot_d + _Clik_gain*_error);
		for (int i=0; i<7; i++)
			output(i+RA_BEGIN) =_qdot_d(i)/_Hz +_Rarm.q(i);
		}
	else
	{
		if (_cnt>= _init_info.t && _cnt <= _init_info.t + _current_task_time) {
			for (int i=0; i<3; i++) {
				_cubic_x(i) = Cubic(_cnt, _init_info.t, _init_info.t+ _current_task_time, _init_info.T_Larm.translation()(i), 0.0, _target_x(i), 0.0);
				_cubic_x(i+3) = Cubic(_cnt, _init_info.t, _init_info.t+ _current_task_time, 0.0, 0.0, _target_x(i+3), 0.0);
			}
			GetPhi(_Larm.T, _init_info.T_Larm, _cubic_x, _lphi);

			if (_cnt == _init_info.t)
				_pre_x = _Larm.T.translation();

			for (int i=0; i<3; i++){
				_xdot_d(i) = (_cubic_x(i)- _pre_x(i))*_Hz;
				_xdot_d(i+3) = -_lphi(i);
				_pre_x(i) = _cubic_x(i);
			}
			for (int i=0; i<3; i++){
				_error(i) = _cubic_x(i) - _Larm.T.translation()(i);
				_error(i+3) = -_lphi(i);
			}
		}
		else if (_cnt > _init_info.t+_current_task_time) {
			_cubic_x = _target_x;
			GetPhi(_Larm.T, _init_info.T_Larm, _cubic_x, _lphi);
			_xdot_d.setZero();
			for (int i=0; i<3; i++){
				_error(i) = _cubic_x(i) - _Larm.T.translation()(i);
				_error(i+3) = -_lphi(i);
			}
		}
		else {
			_Larm.J_inv.setZero();
		}
		_qdot_d = _Larm.J_inv * (_xdot_d + _Clik_gain*_error);
		for (int i=0; i<7; i++)
			output(i+LA_BEGIN) =_qdot_d(i)/_Hz +_Larm.q(i);
	}
}
void UpperCtrl::SET_IK_Parameter(double CLIK_gain, bool relative_motion, bool singular, double sigularity_gain, double singularity_threshold)
{
	_rel = relative_motion;
	_singularity = singular;
	_Clik_gain = CLIK_gain;
	_lambda = sigularity_gain;
        _w0 = singularity_threshold;
}
void UpperCtrl::SET_IK_Local_Target()
{	
	// local target plannar///
	if (_target_state == 0) {
		_pre_target.setZero();
		_pre_time = 0;
	}
	else {
		_pre_target += _current_target;
		_pre_time += _current_task_time;		
	}

	_target_local = _target_global.row(_target_state);
	_arm_type = _target_local(7);

	if (_rel == false)	{			/// ���� ��ǥ�� ����
		for (int i=0; i<3; i++)
		{
			if (_target_local(i) == -1) {
				if (_arm_type)
					_target_local(i) = _init_info.T_Rarm.translation()(i);
				else
					_target_local(i) = _init_info.T_Larm.translation()(i);
			}
			if (_target_local(i+3) == -1) {
				if (_target_local(7) == 0)
					_target_local(i+3) = _init_info.Rarm_euler(i);
				else
					_target_local(i+3) = _init_info.Larm_euler(i);
			}
		}
	}
	else {
		if (_arm_type) {
			for (int i=0; i<3; i++) 
				_target_local(i) = _target_global.row(_target_state)(i)+_init_info.T_Rarm.translation()(i);
		}
		else {
			for (int i=0; i<3; i++) 
				_target_local(i) = _target_global.row(_target_state)(i)+_init_info.T_Larm.translation()(i);
		}
	}
	_target_x = _target_local.head(6);
	_current_task_time = _target_local(6)*_Hz;

	if (_arm_type) {
	cout << "" << endl;
	cout << "Target Value (Rarm)#" << _target_state+1 << endl;
	cout << _target_x(0) <<" " << _target_x(1) <<" " << _target_x(2) <<" "  << _target_x(3) <<" " << _target_x(4) <<" " << _target_x(5) << endl;
	}
	else	{
	cout << "" << endl;
	cout << "Target Value (Larm)#" << _target_state+1 << endl;
	cout << _target_x(0) <<" " << _target_x(1) <<" " << _target_x(2) <<" "  << _target_x(3) <<" " << _target_x(4) <<" " << _target_x(5) << endl;
	}


	_target_state++;
}
void UpperCtrl::SET_IK_Target(MatrixXD planning)
{
	_target_num = planning.col(1).size();
	_target_global = planning;
	_task_time = planning.col(6).sum()*_Hz;
}
void UpperCtrl::FK_compute(VectorXD& output)
{
	if (_cnt == 0)
		_init_info.q = _q;

	FK_solver(_cnt, 0, _duration*_Hz, output);
	_cnt++;
}
void UpperCtrl::FK_solver(int current_time, int init_time, int final_time, VectorXD& output)
{
	for (int i=0; i<JOINT_NUM; i++)
		output(i) = Cubic(current_time, init_time, final_time, _init_info.q(i), 0.0, _target_q(i), 0.0);
}
void UpperCtrl::SET_FK_Target(VectorXD planning)
{
	_target_q = planning;
}
void UpperCtrl::SET_FK_Parameter(double duration)
{
	_duration = duration;	
}
