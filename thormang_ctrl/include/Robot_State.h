
#ifndef __ROBOT_STATE__
#define __ROBOT_STATE__

#include "DefineVar.h"
#include "Walking_Parameter.h"

using namespace math_function;


class Robot_State : virtual public WalkingPara
{
public:
    void ZMP_real(Vector6D& FT_L,Vector6D& FT_R,Vector3D& L_posi,Vector3D& R_posi, Vector3D& ZMP_real_val);
    void GlobalForceMomentCustomPos(Matrix3D &rotMat, Vector3D &distance, Vector6D &LocalForceMoment, Vector6D& GlobalForceMoment);
    void Contactforce_update();
    void updateFT(Vector6D &RFT,Vector6D &LFT);
    void Initial_state_update();
    void Robot_State_para_initialize();
    //void Robot_state_update();
    void change_gyro_frame();
    void change_support_frame();
    //void Contactforce_update();
    //void GlobalForceMomentCustomPos(Matrix3D rotMat, Vector3D distance, Vector6D LocalForceMoment, Vector6D& GlobalForceMoment);
    //void updateFT(Vector6D &LFT, Vector6D &RFT);
    void COM_update();
    void Link_COM_update();

    void lT_R(VectorXD& q, HTransform& lT0_R, HTransform& lT1_R, HTransform& lT2_R, HTransform& lT3_R, HTransform& lT4_R, HTransform& lT5_R);
    void lT_L(VectorXD& q, HTransform& lT0_L, HTransform& lT1_L, HTransform& lT2_L, HTransform& lT3_L, HTransform& lT4_L, HTransform& lT5_L);

    void uT_R(VectorXD& q, HTransform& uT0_R, HTransform& uT1_R, HTransform& uT2_R, HTransform& uT3_R, HTransform& uT4_R, HTransform& uT5_R, HTransform& uT6_R);
    void uT_R_waist(VectorXD& q, HTransform& uT0_R, HTransform& uT1_R, HTransform& uT2_R, HTransform& uT3_R, HTransform& uT4_R, HTransform& uT5_R, HTransform& uT6_R);
    void uT_L(VectorXD& q, HTransform& uT0_L, HTransform& uT1_L, HTransform& uT2_L, HTransform& uT3_L, HTransform& uT4_L, HTransform& uT5_L, HTransform& uT6_L);
    void uT_L_waist(VectorXD& q, HTransform& uT0_L, HTransform& uT1_L, HTransform& uT2_L, HTransform& uT3_L, HTransform& uT4_L, HTransform& uT5_L, HTransform& uT6_L);
    void uT_waist(VectorXD&q, HTransform& uT0, HTransform& UT1);


//	void uJ_L(VectorXD&q,MatrixXD& uJ_L0,MatrixXD& uJ_L1,MatrixXD& uJ_L2,MatrixXD& uJ_L3,MatrixXD& uJ_L4,MatrixXD& uJ_L5,MatrixXD& uJ_L6);
    void Inertia_Matrix_Calculate();
    void Robot_state_update();
    void lJ_L(VectorXD& q, MatrixXD& lJ_L0, MatrixXD& lJ_L1, MatrixXD& lJ_L2, MatrixXD& lJ_L3, MatrixXD& lJ_L4, MatrixXD& lJ_L5);
    void lJ_R(VectorXD& q, MatrixXD& lJ_R0, MatrixXD& lJ_R1, MatrixXD& lJ_R2, MatrixXD& lJ_R3, MatrixXD& lJ_R4, MatrixXD& lJ_R5);

    void COM_Jacobian_update();

    void Jacobian_global(VectorXD& _q);


public:

    Vector6D				_LFT_imp_prev;
    Vector6D				_LFT_imp_prev2;
    Vector6D				_RFT_imp_prev;
    Vector6D				_RFT_imp_prev2;
    Vector6D				_LFT2_imp_prev;
    Vector6D				_LFT2_imp_prev2;
    Vector6D				_RFT2_imp_prev;
    Vector6D				_RFT2_imp_prev2;





};
#endif
