#ifndef __Pattern_Generator__
#define __Pattern_Generator__

#include "DefineVar.h"
#include "Walking_Parameter.h"

using namespace math_function;

class Pattern_generator: virtual public WalkingPara
{
public:
    void Egress(int t_start, int gap, int gap2, VectorXD t_egress);
    void Desired_output(int cnt, int time_parameter_init,int time_parameter_end, Vector6D p_init,Vector6D p_end, Vector6D ori_init, Vector6D ori_end, Vector6D& desired_position,Vector6D& desired_orientation);
    void Foot_trajectory_update2();
    void Foot_swing_trajectory(int cnt, int t_start, int interval, Vector3D& target_p, Vector3D current_p, Vector3D desired_p, double height);
    void Heel_Toe_Motion_pattern();
    void Heel_Toe_Motion();
    void Change_Local_Pattern();
    void Change_Global_Pattern();
    void Trunk_trajectory_update();
    void Foot_trajectory_update();
    void Ref_ZMP_update();
    void Pattern_generator_initialize();
    void GlobaltoLocal_footstep();
    void ZMP_offset_planning(MatrixXD& _foot_step_localframe, MatrixXD& _foot_step_localframe_offset);
    void ZMP_Generator(int Nom_Size, MatrixXD& _foot_step_localframe_offset, MatrixXD& _Ref_ZMP, int planning_step_number);
    void Onestep_ZMP(double T_rest_init, double T_rest_last, double T_Double1,double T_Double2, double T_Total, int step_number,VectorXD& temp_px, VectorXD& temp_py);
    void Onestep_stop_ZMP(double _T_Total,double ZMP_x, double ZMP_y, VectorXD &temp_px,VectorXD& temp_py);
    void Ref_COM_update_local();
    void modified_PreviewControl(int td, int TT);
    void PreviewControl(int k, MatrixXD& K, double dt, int NL, double _COM_init, MatrixXD& Gx, MatrixXD& Gi, VectorXD& Gp_l, MatrixXD& A, MatrixXD& B, VectorXD& px_ref, VectorXD& py_ref, Vector3D xi, Vector3D yi, Vector3D xs, Vector3D ys, Vector3D &xd, Vector3D &yd, double ux_1 , double uy_1 , double &ux, double &uy);
    void equation(double dt, int NL, double _COM_init, MatrixXD& K, MatrixXD& Gi, VectorXD& Gp_l, MatrixXD& Gx, MatrixXD& A, MatrixXD& B);
    MatrixXD dare(MatrixXD& A, MatrixXD& B, MatrixXD& R, MatrixXD& Q);

public:

    Vector3D	_COM_intergral_error;
    Vector3D	_COM_modification;

    Vector6D	_initial_global_support_foot;
    Vector6D	_initial_global_swing_foot;

    Vector6D	_initial_local_support_foot;
    Vector6D	_initial_local_swing_foot;

    Vector6D	_initial_local_support_foot_offset;
    Vector6D	_initial_local_swing_foot_offset;

    Vector3D		_xi;
    Vector3D		_yi;
    Vector3D		_xs;
    Vector3D		_ys;
    Vector3D		_xd;
    Vector3D		_yd;

    Vector3D		_COM_offset;

    double			_ux_1;
    double			_uy_1;
    double			_ux;
    double			_uy;
    VectorXD		_px_ref;
    VectorXD		_py_ref;

    MatrixXD		_K;
    MatrixXD		_Gi;
    VectorXD		_Gp_I;
    MatrixXD		_Gx;
    MatrixXD		_A;
    MatrixXD		_BBB;

    double			start_time;

    Vector6D leg_init; // for taking off car
    Vector6D leg_ori; // for taking off car

    Vector6D p_parameter_init;
    Vector6D p_parameter_end;
    Vector6D ori_parameter_init;
    Vector6D ori_parameter_end;

};
#endif
