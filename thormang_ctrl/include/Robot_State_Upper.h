
#ifndef __ROBOT_STATE_UPPER__
#define __ROBOT_STATE_UPPER__

#include "DefineVar.h"
#include "UpperPara.h"

using namespace math_function;

class Robot_State_Upper : virtual public UpperPara
{
public:
	void Set_Initialize();
	void Set_Joint_Value(VectorXD q);
	void Get_IK_Value();
	void Get_Rarm_Jacobian(MatrixXD& J);
	void Get_Larm_Jacobian(MatrixXD& J);
	void Get_T_Rarm(HTransform& T);
	void Get_T_Larm(HTransform& T);
	void Get_Rarm_angle(Vector3D& angle);
	void Get_Larm_angle(Vector3D& angle);
};
#endif
