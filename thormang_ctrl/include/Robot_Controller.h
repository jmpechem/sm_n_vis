#ifndef __ROBOT_CONTROL__
#define __ROBOT_CONTROL__

#include "DefineVar.h"
#include "Walking_Parameter.h"

using namespace math_function;

class Robot_Control : virtual public WalkingPara
{
public:

public:
    typedef struct {
        double	time;
        Vector6D	FT;
        Vector3D	LFoot_current;
        Vector3D	RFoot_current;
        Matrix3D    LFoot_linear;
        Matrix3D    RFoot_linear;
        Vector3D    LFoot_euler;
        Vector3D    RFoot_euler;
    } Impedance_initial_state;

    void Resolved_momentum_control();
    void Impedance_reference_update();
    void Impedance_update();
    void Impedance_controller();
    void Impedance_control();
    void Impedance_after_update();
    void Impedance_after_update_initialize();

public:
    Vector6D _Impedance_Ref_prev1_L;
    Vector6D _Impedance_Ref_prev2_L;
    Vector6D _Impedance_Ref_prev1_R;
    Vector6D _Impedance_Ref_prev2_R;
    Vector6D _Impedance_Ref_current_L;
    Vector6D _Impedance_Ref_current_R;

    Vector6D _Impedance_Ref_prev1_L2;
    Vector6D _Impedance_Ref_prev2_L2;
    Vector6D _Impedance_Ref_prev1_R2;
    Vector6D _Impedance_Ref_prev2_R2;
    Vector6D _Impedance_Ref_current_L2;
    Vector6D _Impedance_Ref_current_R2;


    bool _Impedance_flag;

    Vector6D _Impedance_T_R_prev2;
    Vector6D _Impedance_T_L_prev2;
    Vector6D _Impedance_T_R_prev1;
    Vector6D _Impedance_T_L_prev1;
    Vector6D _Impedance_T_R_current;
    Vector6D _Impedance_T_L_current;


    Vector6D _Impedance_T_R_desired_prev2;
    Vector6D _Impedance_T_L_desired_prev2;
    Vector6D _Impedance_T_R_desired_prev1;
    Vector6D _Impedance_T_L_desired_prev1;
    Vector6D _Impedance_T_R_desired_current;
    Vector6D _Impedance_T_L_desired_current;

    MatrixXD Xe_LFoot;
    MatrixXD Xe_RFoot;
    MatrixXD Xe_LFoot_Roll;
    MatrixXD Xe_RFoot_Roll;
    MatrixXD Xe_LFoot_Pitch;
    MatrixXD Xe_RFoot_Pitch;


    Vector6D R_position_dot;
    Vector6D L_position_dot;
    Vector6D R_position_dot_desired;
    Vector6D L_position_dot_desired;

    Impedance_initial_state initial_state;

    Vector6D LFT_desired;
    Vector6D RFT_desired;
};
#endif

