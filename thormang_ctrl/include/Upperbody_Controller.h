#include "DefineVar.h"
#include "Robot_State_Upper.h"

using namespace math_function;

class UpperCtrl : public Robot_State_Upper
{

public:
    UpperCtrl();
    ~UpperCtrl();

public:
	void IK_compute(VectorXD& output);
	void IK_solver(VectorXD& output);
	void IK_solver_basic(VectorXD& output);
	//void IK_solver_singularity(VectorXD& output);

	void SET_IK_Initialize();
	void SET_IK_Parameter(double CLIK_gain, bool relative_motion, bool singuarity, double sigularity_gain, double singularity_threshold);
	void SET_IK_Target(MatrixXD planning);
	void SET_IK_Local_Target();
	
	void FK_compute(VectorXD& output);
	void FK_solver(int current_time, int init_time, int final_time, VectorXD& output);
	void SET_FK_Target(VectorXD planning);
	void SET_FK_Parameter(double duration);


};
