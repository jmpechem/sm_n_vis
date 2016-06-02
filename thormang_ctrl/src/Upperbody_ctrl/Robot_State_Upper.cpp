#include "Robot_State_Upper.h"
#include "UpperPara.h"

void Robot_State_Upper::Set_Initialize()
{
	set_user_value();
	Upperpara_initialize();
}
void Robot_State_Upper::Set_Joint_Value(VectorXD q)
{
	_q=q;
}
void Robot_State_Upper::Get_IK_Value()
{
	Get_T_Rarm(_Rarm.T);
	Get_T_Larm(_Larm.T);

	Get_Larm_angle(_Larm.EulerAngle);
	Get_Rarm_angle(_Rarm.EulerAngle);

	Get_Rarm_Jacobian(_Rarm.J);
	Get_Larm_Jacobian(_Larm.J);

	for (int i=0; i<7; i++) {
		_Rarm.q(i) = _q(i+RA_BEGIN);
		_Larm.q(i) = _q(i+LA_BEGIN);
	}

}
void Robot_State_Upper::Get_Rarm_Jacobian(MatrixXD& J)
{
	MatrixXD J_G_R_temp(6,7);
	J_G_R_temp.setZero();
	Matrix3D R_R = _Rarm.T.linear();
	VectorXD q = _q;

	J_G_R_temp(0,0) = -l_6*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+l_8*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-l_9*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))+l_7*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0));
	J_G_R_temp(0,1) = -l_8*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-l_9*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-l_7*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))-l_6*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0));
	J_G_R_temp(0,2) = l_6*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))-l_8*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+l_9*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)));
	J_G_R_temp(0,3) = l_8*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+l_9*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)));
	J_G_R_temp(1,1) = l_8*(cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))+cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2)))+l_9*(cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+l_7*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))-l_6*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2));
	J_G_R_temp(1,2) = -l_6*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2))+l_8*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+2))-l_9*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+2));
	J_G_R_temp(1,3) = l_8*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))-l_9*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)));
	J_G_R_temp(2,0) = l_6*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-l_8*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+l_9*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+l_7*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0));
	J_G_R_temp(2,1) = l_8*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+l_9*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+l_7*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))+l_6*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2));
	J_G_R_temp(2,2) = l_6*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))-l_8*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+l_9*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)));
	J_G_R_temp(2,3) = l_8*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))+l_9*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)));
	J_G_R_temp(3,1) = cos(q(RA_BEGIN+0));
	J_G_R_temp(3,2) = cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0));
	J_G_R_temp(3,3) = cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2));
	J_G_R_temp(3,4) = sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0));
	J_G_R_temp(3,5) = -sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)));
	J_G_R_temp(3,6) = cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	J_G_R_temp(4,0) = -1.0;
	J_G_R_temp(4,2) = sin(q(RA_BEGIN+1)-pi*(1.0/2.0));
	J_G_R_temp(4,3) = -cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2));
	J_G_R_temp(4,4) = cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2));
	J_G_R_temp(4,5) = sin(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+4))*sin(q(RA_BEGIN+2));
	J_G_R_temp(4,6) = -sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2))*sin(q(RA_BEGIN+4)))+cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)));
	J_G_R_temp(5,1) = sin(q(RA_BEGIN+0));
	J_G_R_temp(5,2) = -cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0));
	J_G_R_temp(5,3) = cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2));
	J_G_R_temp(5,4) = sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0));
	J_G_R_temp(5,5) = -sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)));
	J_G_R_temp(5,6) = sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0))))+cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)));

	Matrix6D Jp7toJpe_R;
	Jp7toJpe_R.setIdentity();

	Vector3D frame7toee_R;
	frame7toee_R.setZero();

	frame7toee_R(2) = -0.28;

	Matrix3D tmp_matrix_R;
	tmp_matrix_R.setZero();

	Skew(-R_R*frame7toee_R,tmp_matrix_R);
	Jp7toJpe_R.block<3,3>(0,3) =tmp_matrix_R;
	J=Jp7toJpe_R*J_G_R_temp;
}
void Robot_State_Upper::Get_Larm_Jacobian(MatrixXD& J)
{
	MatrixXD J_G_L_temp(6,7);
	J_G_L_temp.setZero();

	Matrix3D R_L = _Larm.T.linear();
	VectorXD q = _q;

	J_G_L_temp(0,0) = l_6*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-l_8*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-l_9*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-l_7*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0));
	J_G_L_temp(0,1) = -l_8*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+l_9*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+l_7*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))-l_6*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0));
	J_G_L_temp(0,2) = -l_6*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+l_8*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+l_9*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)));
	J_G_L_temp(0,3) = -l_8*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+l_9*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)));
	J_G_L_temp(1,1) = -l_8*(cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))+cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2)))+l_9*(cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+l_7*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))+l_6*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2));
	J_G_L_temp(1,2) = l_6*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2))-l_8*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+2))-l_9*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+2));
	J_G_L_temp(1,3) = -l_8*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))-l_9*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)));
	J_G_L_temp(2,0) = l_6*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-l_8*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-l_9*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+l_7*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0));
	J_G_L_temp(2,1) = -l_8*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+l_9*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+l_7*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))-l_6*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2));
	J_G_L_temp(2,2) = l_6*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))-l_8*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))-l_9*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)));
	J_G_L_temp(2,3) = l_8*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-l_9*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)));
	J_G_L_temp(3,1) = cos(q(LA_BEGIN+0));
	J_G_L_temp(3,2) = -cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0));
	J_G_L_temp(3,3) = cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2));
	J_G_L_temp(3,4) = sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0));
	J_G_L_temp(3,5) = -sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)));
	J_G_L_temp(3,6) = cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	J_G_L_temp(4,0) = 1.0;
	J_G_L_temp(4,2) = sin(q(LA_BEGIN+1)+pi*(1.0/2.0));
	J_G_L_temp(4,3) = -cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2));
	J_G_L_temp(4,4) = cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2));
	J_G_L_temp(4,5) = sin(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+4))*sin(q(LA_BEGIN+2));
	J_G_L_temp(4,6) = -sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2))*sin(q(LA_BEGIN+4)))+cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)));
	J_G_L_temp(5,1) = -sin(q(LA_BEGIN+0));
	J_G_L_temp(5,2) = -cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0));
	J_G_L_temp(5,3) = -cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2));
	J_G_L_temp(5,4) = -sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0));
	J_G_L_temp(5,5) = sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)));
	J_G_L_temp(5,6) = -sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0))))-cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)));

	Matrix6D Jp7toJpe_L;
	Jp7toJpe_L.setIdentity();

	Vector3D frame7toee_L;
	frame7toee_L.setZero();

	frame7toee_L(2) = -0.28;

	Matrix3D tmp_matrix_L;
	tmp_matrix_L.setZero();

	Skew(-R_L*frame7toee_L,tmp_matrix_L);
	Jp7toJpe_L.block<3,3>(0,3) =tmp_matrix_L;

	J=Jp7toJpe_L*J_G_L_temp;
}
void Robot_State_Upper::Get_T_Rarm(HTransform& T)
{
	Vector3D r_temp;
	r_temp.setZero();
	Matrix3D R_R;
	VectorXD q = _q;

	R_R(0,0) = sin(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))))-cos(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	R_R(0,1) = -cos(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))))-sin(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	R_R(0,2) = -cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))-sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	R_R(1,0) = cos(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+4))*sin(q(RA_BEGIN+2)))+sin(q(RA_BEGIN+6))*(cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2))*sin(q(RA_BEGIN+4)))+sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2))));
	R_R(1,1) = sin(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+4))*sin(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+6))*(cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2))*sin(q(RA_BEGIN+4)))+sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2))));
	R_R(1,2) = sin(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+4))*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+2))*sin(q(RA_BEGIN+4)))-cos(q(RA_BEGIN+5))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)));
	R_R(2,0) = sin(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))))-cos(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	R_R(2,1) = -cos(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))))-sin(q(RA_BEGIN+6))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))));
	R_R(2,2) = -sin(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+4))*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0))))-cos(q(RA_BEGIN+5))*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)));

	r_temp(0) = l_6*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-l_8*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+l_9*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(cos(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))-sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2))*sin(q(RA_BEGIN+0)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+0)))+l_7*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+0));
	r_temp(1) = -l_4+l_8*(sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+l_9*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+2)))+l_7*sin(q(RA_BEGIN+1)-pi*(1.0/2.0))+l_6*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+2));
	r_temp(2) = l_5+l_6*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-l_8*(cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))+cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))+l_9*(sin(q(RA_BEGIN+3)+pi*(1.0/4.0))*(sin(q(RA_BEGIN+0))*sin(q(RA_BEGIN+2))+sin(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0))*cos(q(RA_BEGIN+2)))-cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+3)+pi*(1.0/4.0))*cos(q(RA_BEGIN+0)))-l_7*cos(q(RA_BEGIN+1)-pi*(1.0/2.0))*cos(q(RA_BEGIN+0));

	Vector3D	temp;
	temp.setZero();

	temp(2) = -0.28;

	T.linear() = R_R;
	T.translation() = (R_R*temp) + r_temp;
}
void Robot_State_Upper::Get_T_Larm(HTransform& T)
{
	Vector3D r_temp;
	Matrix3D R_L;
	r_temp.setZero();
	VectorXD q = _q;

	R_L(0,0) = sin(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))))-cos(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	R_L(0,1) = -cos(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))))-sin(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	R_L(0,2) = -cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	R_L(1,0) = cos(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+4))*sin(q(LA_BEGIN+2)))+sin(q(LA_BEGIN+6))*(cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2))*sin(q(LA_BEGIN+4)))+sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2))));
	R_L(1,1) = sin(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+4))*sin(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+6))*(cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2))*sin(q(LA_BEGIN+4)))+sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2))));
	R_L(1,2) = sin(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+4))*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+2))*sin(q(LA_BEGIN+4)))-cos(q(LA_BEGIN+5))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)));
	R_L(2,0) = -sin(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))))+cos(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	R_L(2,1) = cos(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))))+sin(q(LA_BEGIN+6))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))));
	R_L(2,2) = sin(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+4))*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0))))+cos(q(LA_BEGIN+5))*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)));

	r_temp(0) = -l_6*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+l_8*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))+l_9*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(cos(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))+sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2))*sin(q(LA_BEGIN+0)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+0)))-l_7*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+0));
	r_temp(1) = l_4-l_8*(sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+l_9*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+2)))+l_7*sin(q(LA_BEGIN+1)+pi*(1.0/2.0))-l_6*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+2));
	r_temp(2) = l_5+l_6*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-l_8*(cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))-cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-l_9*(sin(q(LA_BEGIN+3)-pi*(1.0/4.0))*(sin(q(LA_BEGIN+0))*sin(q(LA_BEGIN+2))-sin(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0))*cos(q(LA_BEGIN+2)))+cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+3)-pi*(1.0/4.0))*cos(q(LA_BEGIN+0)))-l_7*cos(q(LA_BEGIN+1)+pi*(1.0/2.0))*cos(q(LA_BEGIN+0));

	Vector3D	temp;
	temp.setZero();
	temp(2) = -0.28;

	T.linear() = R_L;
	T.translation() =  (R_L* temp) + r_temp;
}
void Robot_State_Upper::Get_Rarm_angle(Vector3D& angle)
{
	Rot2euler(_Rarm.T.linear(), angle);
}
void Robot_State_Upper::Get_Larm_angle(Vector3D& angle)
{
	Rot2euler(_Larm.T.linear(), angle);
}
