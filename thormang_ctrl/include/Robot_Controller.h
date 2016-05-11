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
    } Impedance_initial_state;

    void Resolved_momentum_control();
    void Impedance_reference_update();
    void Impedance_update();
    void Impedance_controller();
    void Impedance_control();
public:

    Vector6D _Impedance_Ref_prev1_L;
    Vector6D _Impedance_Ref_prev2_L;
    Vector6D _Impedance_Ref_prev1_R;
    Vector6D _Impedance_Ref_prev2_R;
    bool _Impedance_flag;
    Vector6D _Impedance_T_R_prev1;
    Vector6D _Impedance_T_L_prev1;
    Vector6D _Impedance_T_R_current;
    Vector6D _Impedance_T_L_current;

    Vector6D _Impedance_T_R_desired_prev1;
    Vector6D _Impedance_T_L_desired_prev1;
    Vector6D _Impedance_T_R_desired_current;
    Vector6D _Impedance_T_L_desired_current;



    Vector6D R_position_dot;
    Vector6D L_position_dot;
    Vector6D R_position_dot_desired;
    Vector6D L_position_dot_desired;

    Impedance_initial_state initial_state;

    Vector6D LFT_desired;
    Vector6D RFT_desired;
};
#endif

