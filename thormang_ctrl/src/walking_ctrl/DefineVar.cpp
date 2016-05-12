#include "DefineVar.h"

namespace math_function{

    /////////////////////////////////////////////////////////////////////
    // Skew
    //
    void
        Skew(Vector3D src, Matrix3D& skew)
    {
        skew.setZero();
        skew(0, 1) = -src[2];
        skew(0, 2) = src[1];
        skew(1, 0) = src[2];
        skew(1, 2) = -src[0];
        skew(2, 0) = -src[1];
        skew(2, 1) = src[0];
    }

    float
        Cubic_dot(double rT, float rT_0, float rT_f, float rx_0, float rx_dot_0, float rx_f, float rx_dot_f, float Hz)
    {
        double rx_t ;
        if(rT<rT_0)
        {
            rx_t = 0.0;
        }
        else if(rT>=rT_0 && rT<rT_f)
        {
            rT_f = rT_f/Hz;
            rT_0 = rT_0/Hz;
            rT = rT/Hz;

            rx_t = rx_dot_0
                + 2*(3 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0)) - 2 * rx_dot_0/(rT_f-rT_0) - rx_dot_f/(rT_f-rT_0))*(rT-rT_0)
                + 3*(-2 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0) * (rT_f-rT_0)) + (rx_dot_0 + rx_dot_f)/((rT_f-rT_0) * (rT_f-rT_0)))*(rT-rT_0)*(rT-rT_0);
        }
        else
        {
            rx_t = 0.0;
        }
        return (rx_t);
    }



    float
        Cubic(double rT, float rT_0, float rT_f, float rx_0, float rx_dot_0, float rx_f, float rx_dot_f)
    {
        double rx_t ;
        if(rT<rT_0)
        {
            rx_t = rx_0;
        }
        else if(rT>=rT_0 && rT<rT_f)
        {
            rx_t = rx_0 + rx_dot_0*(rT-rT_0)
                + (3 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0)) - 2 * rx_dot_0/(rT_f-rT_0) - rx_dot_f/(rT_f-rT_0))*(rT-rT_0)*(rT-rT_0)
                + (-2 * (rx_f - rx_0)/((rT_f-rT_0) * (rT_f-rT_0) * (rT_f-rT_0)) + (rx_dot_0 + rx_dot_f)/((rT_f-rT_0) * (rT_f-rT_0)))*(rT-rT_0)*(rT-rT_0)*(rT-rT_0);
        }
        else
        {
            rx_t = rx_f;
        }
        return (rx_t);
    }

    Matrix3D
        Rotate_with_X(double rAngle)
    {
        Matrix3D _Rotate_wth_X;

        _Rotate_wth_X(0, 0) = 1.0;
        _Rotate_wth_X(1, 0) = 0.0;
        _Rotate_wth_X(2, 0) = 0.0;

        _Rotate_wth_X(0, 1) = 0.0;
        _Rotate_wth_X(1, 1) = cos(rAngle);
        _Rotate_wth_X(2, 1) = sin(rAngle);

        _Rotate_wth_X(0, 2) = 0.0;
        _Rotate_wth_X(1, 2) = -sin(rAngle);
        _Rotate_wth_X(2, 2) = cos(rAngle);

        return(_Rotate_wth_X);
    }

    void
        Globalframe(HTransform& trunk, HTransform& reference, HTransform& new_trunk)
    {
        Matrix3D temp;
        temp = reference.linear().transpose();

        new_trunk.linear() = temp*trunk.linear();
        new_trunk.translation() = temp*(trunk.translation() - reference.translation()); // ŸÆÁ÷±îÁø ±Û·Î¹ú ÁÂÇ¥°èÀÓ
    }


    void
        GlobalGyroframe(HTransform& trunk, HTransform& reference, HTransform& new_trunk)
    {

        Vector3D ref_ang;
        Rot2euler(reference.linear(),ref_ang);

        Matrix3D temp;
        temp = Rotate_with_Z(-ref_ang(2));

        new_trunk.linear() = temp*trunk.linear();
        new_trunk.translation() = temp*(trunk.translation() - reference.translation()); // ŸÆÁ÷±îÁø ±Û·Î¹ú ÁÂÇ¥°èÀÓ
    }


    void
        Rot2euler(Matrix3D Rot, Vector3D & angle)
    {
        double beta;

        beta = -asin(Rot(2,0));

        if(abs(beta) < 90*DEGREE)
            beta = beta;
        else
            beta = 180*DEGREE-beta;

        angle(0) = atan2(Rot(2,1),Rot(2,2)+1E-37); //x
        angle(2) = atan2(Rot(1,0),Rot(0,0)+1E-37); //z
        angle(1) = beta; //y
    }



    /////////////////////////////////////////////////////////////////////
    // Rotate_with_Y
    //
    Matrix3D
        Rotate_with_Y(double rAngle)
    {
        Matrix3D _Rotate_wth_Y(3, 3);

        _Rotate_wth_Y(0, 0) = cos(rAngle);
        _Rotate_wth_Y(1, 0) = 0.0;
        _Rotate_wth_Y(2, 0) = -sin(rAngle);

        _Rotate_wth_Y(0, 1) = 0.0;
        _Rotate_wth_Y(1, 1) = 1.0;
        _Rotate_wth_Y(2, 1) = 0.0;

        _Rotate_wth_Y(0, 2) = sin(rAngle);
        _Rotate_wth_Y(1, 2) = 0.0;
        _Rotate_wth_Y(2, 2) = cos(rAngle);

        return(_Rotate_wth_Y);
    }

    /////////////////////////////////////////////////////////////////////
    // Rotate_with_Z
    //
    Matrix3D
        Rotate_with_Z(double rAngle)
    {
        Matrix3D _Rotate_wth_Z(3, 3);

        _Rotate_wth_Z(0, 0) = cos(rAngle);
        _Rotate_wth_Z(1, 0) = sin(rAngle);
        _Rotate_wth_Z(2, 0) = 0.0;

        _Rotate_wth_Z(0, 1) = -sin(rAngle);
        _Rotate_wth_Z(1, 1) = cos(rAngle);
        _Rotate_wth_Z(2, 1) = 0.0;

        _Rotate_wth_Z(0, 2) = 0.0;
        _Rotate_wth_Z(1, 2) = 0.0;
        _Rotate_wth_Z(2, 2) = 1.0;

        return(_Rotate_wth_Z);
    }



    void
        Globalposition(Vector3D& target, HTransform& reference, Vector3D& new_target)
    {
        Matrix3D temp;
        temp = reference.linear().transpose();

        new_target = temp*(target-reference.translation());

    }

}