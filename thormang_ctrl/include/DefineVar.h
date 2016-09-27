#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace Eigen;

//type of user data
typedef double	rScalar;
typedef Eigen::Transform<rScalar, 3, Eigen::Isometry> Isometry3D;
typedef Eigen::Matrix<rScalar, -1, 1>	VectorXD;
typedef Eigen::Matrix<rScalar, 2, 1>	Vector2D;
typedef Eigen::Matrix<rScalar, 3, 1>	Vector3D;
typedef Eigen::Matrix<rScalar, 4, 1>	Vector4D;
typedef Eigen::Matrix<rScalar, 6, 1>	Vector6D;
typedef Eigen::Matrix<rScalar, -1, -1>	MatrixXD;
typedef Eigen::Matrix<rScalar, 2, 2>	Matrix2D;
typedef Eigen::Matrix<rScalar, 3, 3>	Matrix3D;
typedef Eigen::Matrix<rScalar, 4, 4>	Matrix4D;
typedef Eigen::Matrix<rScalar, 6, 6>	Matrix6D;
typedef Eigen::Matrix<rScalar, 3, 6>	Matrix36D;
typedef Eigen::Matrix<rScalar, 6, 3>	Matrix63D;
typedef Eigen::Matrix<rScalar, 3, -1>	Matrix3XD;
typedef Eigen::Matrix<rScalar, -1, 3>	MatrixX3D;
typedef Eigen::Matrix<rScalar, 6, -1>	Matrix6XD;
typedef Eigen::Matrix<rScalar, -1, 6>	MatrixX6D;
typedef Eigen::Transform<rScalar, 3, Eigen::Isometry> HTransform;
typedef Eigen::Matrix<rScalar, -1, -1> dMatrix;
typedef Eigen::Matrix<rScalar, -1, 1>  dVector;

// Thormang model information & constant value
#define l_0 0.105
#define l_1 0.1829
#define l_2 0.3
#define l_3 0.3 
#define l_4 0.21 
#define l_5 0.2785 
#define l_6 0.02995
#define l_7 0.264
#define l_8 0.03
#define l_9 0.252

//for modified version Robot size
#define l_0 0.105
#define l_1 0.1829
#define l_2_x 0.1554
#define l_2_z 0.339
#define l_3_x 0.06
#define l_3_z 0.368




#define pi			3.141592653589793238462
#define Gravity	9.81
#define DEGREE	(0.01745329251994329576923690768489)
#define RADIAN	(57.295779513082320876798154814096) //(180.0/M_PI)

#define WA_BEGIN 0
#define RA_BEGIN 2
#define LA_BEGIN 9
#define RF_BEGIN 16
#define LF_BEGIN 22

//basic math function

namespace math_function{
    void Skew(Vector3D src, Matrix3D& skew);
    float Cubic(double rT, float rT_0, float rT_f, float rx_0, float rx_dot_0, float rx_f, float rx_dot_f);
    float Cubic_dot(double rT, float rT_0, float rT_f, float rx_0, float rx_dot_0, float rx_f, float rx_dot_f, float Hz);
    Matrix3D Rotate_with_X(double rAngle);
    Matrix3D Rotate_with_Y(double rAngle);
    Matrix3D Rotate_with_Z(double rAngle);
    void GlobalGyroframe(HTransform& trunk, HTransform& reference, HTransform& new_trunk);
    void Globalframe(HTransform& trunk, HTransform& reference, HTransform& new_trunk);
    void Rot2euler(Matrix3D Rot, Vector3D & angle);
    void Globalposition(Vector3D& target, HTransform& reference, Vector3D& new_target);
    void GetPhi(HTransform& RotationMtx, HTransform& active_R, Vector6D& ctrl_pos_ori, Vector3D& phi);
    VectorXD ClampMaxAbs(VectorXD a, double b);
}
