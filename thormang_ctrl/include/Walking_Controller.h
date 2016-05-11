#include "DefineVar.h"
#include "FootPlanning.h"
#include "Robot_State.h"
#include "Pattern_Generator.h"
#include "Robot_Controller.h"

using namespace math_function;

class WalkingCtrl : public Foot_Planning, public Robot_State, public Pattern_generator, public Robot_Control
{


public:
    WalkingCtrl();
    ~WalkingCtrl();


public:

    Vector3D Base_global_position;


public:
    void Init_walking_pose(VectorXD& output);

    void Step_time_update();
    void Step_count(MatrixXD& foot_planning);
    void getdata(VectorXD& _q_robot, Vector6D& _L_Ft_robot, Vector6D& _R_Ft_robot, Vector3D& _Gyro_Base_robot);
    void update_robot_data();
    void _initialize();
    void compute(VectorXD& output);
    void _init_state_update();
    void InverseKinematics(Vector3D P_wt, Vector3D P_wr5, Vector3D P_wl5, Matrix3D R_wt, Matrix3D R_wr5, Matrix3D R_wl5, VectorXD& qd);
};
