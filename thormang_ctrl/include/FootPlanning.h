#ifndef __FOOT_PLANNING__
#define __FOOT_PLANNING__

#include "DefineVar.h"
#include "Walking_Parameter.h"

using namespace math_function;

class Foot_Planning : virtual public WalkingPara
{
public:

public:
    void plan_foot_step(Vector3D& _scan_data, MatrixXD& _foot_step,bool _Step_Planning_flag);
    void calculate_foot_step_total(Vector3D& _scan_data, MatrixXD& _foot_step);
    void calculate_foot_step_separate(Vector3D& _scan_data,MatrixXD& _foot_step);
};
#endif
