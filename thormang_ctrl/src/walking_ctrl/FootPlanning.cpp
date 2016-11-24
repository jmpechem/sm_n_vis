#include "FootPlanning.h"

void Foot_Planning::plan_foot_step(Vector3D& _scan_data, MatrixXD& _foot_step, bool _Step_Planning_flag)
{
    if(_scan_data(0) == 0 && _scan_data(1) == 0 && _scan_data(2) == 0)
    {

       // _foot_step.resize(1,7);
        _foot_step.resize(80,7);
        _foot_step.setZero();

        for (int i=0; i<40 ;i++)
        {
           //int i = 0;
        _foot_step(2*i+0,0) = 0.0;//0.2*i+0.1;	//0.0;//-0.0491398 ;//0.2*i+0.2;
        _foot_step(2*i+0,1) = -0.127794;
        _foot_step(2*i+0,2) = 0.0;
        _foot_step(2*i+0,6) = 1;

        _foot_step(2*i+1,0) = 0.0;//0.2*i+0.2;	//0.0;//-0.0491398 ;//0.2*i+0.2;
        _foot_step(2*i+1,1) = 0.127794;
        _foot_step(2*i+1,2) = 0.0;
        _foot_step(2*i+1,6) = 0;
        }

/*
        _foot_step.resize(1,7);
        _foot_step.setZero();

        _foot_step(0,0) = 0.0;
        _foot_step(0,1) = 0.127794;
        _foot_step(0,6) = 0;
*/
        //}
    }
    else
    {
        if(_Step_Planning_flag == false)
            calculate_foot_step_total(_scan_data,_foot_step);
        else
            calculate_foot_step_separate(_scan_data,_foot_step);
    }

    _step_total_number = _foot_step.col(1).size();
//	cout << "planning_foot_step" << _foot_step << endl;
}

void Foot_Planning::calculate_foot_step_total(Vector3D& _scan_data, MatrixXD& _foot_step)
{

    double x = _scan_data(0);
    double y = _scan_data(1);
    double alpha = _scan_data(2);

    double initial_rot;

    initial_rot= atan2(y,x);

    double final_rot = 0.0;

    double initial_drot = 0.0;
    double final_drot = 0.0;

    if(initial_rot > 0.0)
        initial_drot = 10*DEGREE;
    else
        initial_drot = -10*DEGREE;

    int initial_total_step_number = initial_rot/initial_drot;
    double initial_residual_angle = initial_rot-initial_total_step_number*initial_drot;

    final_rot = alpha-initial_rot;
    if(final_rot > 0.0)
        final_drot = 10*DEGREE;
    else
        final_drot = -10*DEGREE;

    int final_total_step_number = final_rot/final_drot;
    double final_residual_angle = final_rot-final_total_step_number*final_drot;

    double L = sqrt(x*x+y*y);
    double dlength = 0.10;
    int middle_total_number = L/(dlength);
    double middle_residual_length = L-middle_total_number*(dlength);

    /////////////////// foot_Step »çÀÌÁî °è»ê///////////////

    int temp_size;


    int del_size;

    del_size = 1;
    temp_size = initial_total_step_number*del_size+middle_total_number *del_size+final_total_step_number*del_size;

    if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
    {
        if(initial_total_step_number%2==0)
            temp_size = temp_size+2*del_size;
        else
        {
            if(abs(initial_residual_angle)>= 0.0001)
                temp_size = temp_size+3*del_size;
            else
                temp_size = temp_size+del_size;
        }
    }

    if(middle_total_number!=0 || abs(middle_residual_length)>=0.0001)
    {
        if(middle_total_number%2==0)
            temp_size = temp_size+2*del_size;
        else
        {
            if(abs(middle_residual_length)>= 0.0001)
                temp_size = temp_size+3*del_size;
            else
                temp_size = temp_size+del_size;
        }
    }

    if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
    {
        if(abs(final_residual_angle)>= 0.0001)
            temp_size = temp_size+2*del_size;
        else
            temp_size = temp_size+del_size;
    }


    _foot_step.resize(temp_size,7);
    _foot_step.setZero();
    ///////////////////»çÀÌÁî °è»ê ³¡///// ÃÊ±â °¢µµ ÈžÀü œÃÀÛ //////

    int index = 0;
    int temp = -1;

    if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
    {
        for (int i =0 ; i<initial_total_step_number; i++)
        {
            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin((i+1)*initial_drot);
            _foot_step(index,1) = -temp*0.127794*cos((i+1)*initial_drot);
            _foot_step(index,5) = (i+1)*initial_drot;
            _foot_step(index,6) = 0.5+0.5*temp;
            index++;

        }

        if(temp==1) // ¿Þ¹ßÁöÁö·Î ³¡³µÀžžé
        {
            if(abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
                _foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
                _foot_step(index,6) = 0.5+0.5*temp;
                index++;

                temp *= -1;

                _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
                _foot_step(index,6) = 0.5+0.5*temp;
                index++;

            }
            else
            {
                temp *= -1;

                _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
                _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
                _foot_step(index,6) = 0.5+0.5*temp;
                index++;
            }
        }
        else if(temp==-1) // ¿Àž¥¹ßÁöÁö·Î ³¡³µÀžžé
        {
            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
            _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
            _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
            _foot_step(index,6) = 0.5+0.5*temp;
            index++;

            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
            _foot_step(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
            _foot_step(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
            _foot_step(index,6) = 0.5+0.5*temp;
            index++;
        }
    }

    //////////////////ÃÊ±â ÈžÀü ³¡ ÀÌµ¿ œÃÀÛ ////////////////////////
    int temp2 = -1;

    if(middle_total_number!=0 || abs(middle_residual_length)>=0.0001)
    {
        for (int i =0 ; i<middle_total_number; i++)
        {
            temp2 *= -1;

            _foot_step(index,0) = cos(initial_rot)*(dlength*(i+1))+temp2*sin(initial_rot)*(0.127794);
            _foot_step(index,1) = sin(initial_rot)*(dlength*(i+1))-temp2*cos(initial_rot)*(0.127794);
            _foot_step(index,5) = initial_rot;
            _foot_step(index,6) = 0.5+0.5*temp2;
            index++;
        }

        if(temp2==1) // ¿Þ¹ßÁöÁö·Î ³¡³µÀžžé
        {
            if(abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
                _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
                _foot_step(index,5) = initial_rot;
                _foot_step(index,6) = 0.5+0.5*temp2;
                index++;

                temp2 *= -1;

                _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
                _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
                _foot_step(index,5) = initial_rot;
                _foot_step(index,6) = 0.5+0.5*temp2;
                index++;

                temp2 *= -1;

                _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
                _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
                _foot_step(index,5) = initial_rot;
                _foot_step(index,6) = 0.5+0.5*temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
                _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
                _foot_step(index,5) = initial_rot;
                _foot_step(index,6) = 0.5+0.5*temp2;
                index++;
            }
        }
        else if(temp2==-1)
        {
            temp2 *= -1;

            _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
            _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
            _foot_step(index,5) = initial_rot;
            _foot_step(index,6) = 0.5+0.5*temp2;
            index++;

            temp2 *= -1;

            _foot_step(index,0) = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
            _foot_step(index,1) = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
            _foot_step(index,5) = initial_rot;
            _foot_step(index,6) = 0.5+0.5*temp2;
            index++;
        }
    }
/*
    cout << "middle total number" << middle_total_number << endl;
    cout << "middle residual length" << middle_residual_length << endl;
    cout << "total foot step1" << _foot_step << endl;*/


    //////////////////Á÷Áø ³¡ ž¶Áöž· ÈžÀü œÃÀÛ ///////////////////

    double final_position_x = cos(initial_rot)*(dlength*(middle_total_number)+middle_residual_length);
    double final_position_y = sin(initial_rot)*(dlength*(middle_total_number)+middle_residual_length);

    int temp3 = -1;

    if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
    {
        for (int i =0 ; i<final_total_step_number; i++)
        {
            temp3 *= -1;

            _foot_step(index,0) = final_position_x+temp3*0.127794*sin((i+1)*final_drot+initial_rot);
            _foot_step(index,1) = final_position_y-temp3*0.127794*cos((i+1)*final_drot+initial_rot);
            _foot_step(index,5) = (i+1)*final_drot+initial_rot;
            _foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }

        if(abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            _foot_step(index,0) = final_position_x+temp3*0.127794*sin(alpha);
            _foot_step(index,1) = final_position_y-temp3*0.127794*cos(alpha);
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+0.5*temp3;
            index++;

            temp3 *= -1;

            _foot_step(index,0) = final_position_x+temp3*0.127794*sin(alpha);
            _foot_step(index,1) = final_position_y-temp3*0.127794*cos(alpha);
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            _foot_step(index,0) = final_position_x+temp3*0.127794*sin(alpha);
            _foot_step(index,1) = final_position_y-temp3*0.127794*cos(alpha);
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+0.5*temp3;
            index++;
        }
    }
}

void Foot_Planning::calculate_foot_step_separate(Vector3D& _scan_data,MatrixXD& _foot_step)
{

    double x = _scan_data(0);
    double y = _scan_data(1);
    double alpha = _scan_data(2);

    double dx = 0.1;
    double dy = 0.05;
    double dtheta = 10.0*DEGREE;
    if(x<0.0)
        dx = -0.1;
    if(y<0.0)
        dy = -0.05;
    if(alpha<0.0)
        dtheta = -10.0*DEGREE;

    int x_number = x/dx;
    int y_number = y/dy;
    int theta_number = alpha/dtheta;

    double x_residual = x-x_number*dx;
    double y_residual = y-y_number*dy;
    double theta_residual = alpha-theta_number*dtheta;

    //////////////// size °è»ê ///////////////
    int foot_size = 0;
    int temp = -1;

    if(x_number!=0 || abs(x_residual)>=0.001)
    {
        temp *= -1;
        foot_size += 1;

        for(int i=0;i<x_number;i++)
        {
            temp *= -1;
        }
        foot_size += x_number;

        if(abs(x_residual)>=0.001)
        {
            temp *= -1;
            temp *= -1;
            foot_size += 2;
        }
        else
        {
            temp *= -1;
            foot_size += 1;
        }
    }

    if(y_number!=0 || abs(y_residual)>=0.001)
    {
        if(x==0) // xœºÅÜÀÌ ŸøÀ» ¶§ŽÂ y¹æÇâ ºÎÈ£¿¡ µû¶ó Ã¹¹ß °áÁ€
        {
            if(y>=0)
                temp = -1;
            else
                temp = 1;
            temp *= -1;
            foot_size += 1;
        }

        if(y>=0 && temp==-1) // Ã¹¹ß Á¶°Ç
        {
            foot_size += 1;
        }
        else if(y<0 && temp==1)
        {
            foot_size += 1;
        }

        foot_size += 2*y_number;

        if(abs(y_residual)>=0.001)
        {
            foot_size += 2;
        }
    }

    if(theta_number!=0 || abs(theta_residual)>= 0.02)
    {
        foot_size += theta_number;

        if(abs(theta_residual) >= 0.02)
        {
            foot_size += 2;
        }
        else
        {
            foot_size += 1;
        }
    }


    _foot_step.resize(foot_size, 7);
    _foot_step.setZero();

    /////////////// x¹æÇâ Á÷Áø //////////////////

    //int temp = -1;
    temp = -1;
    int index = 0;

    if(x_number!=0 || abs(x_residual)>=0.001)
    {
        temp *= -1;	// Ã¹¹ßÀº ÁŠÀÚž®

        // ÂøÁö¹ß °è»ê
        _foot_step(index,0) = 0;
        _foot_step(index,1) = -temp*0.127794;
        _foot_step(index,6) = 0.5+temp*0.5;
        index++;

        for(int i=0;i<x_number;i++)
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = (i+1)*dx;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }

        if(abs(x_residual)>=0.001)
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;

            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
        else
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
    }

    /////////////// y¹æÇâ »çÀÌµå //////////////////
    if(y_number!=0 || abs(y_residual)>=0.001)
    {
        if(x==0) // xœºÅÜÀÌ ŸøÀ» ¶§ŽÂ y¹æÇâ ºÎÈ£¿¡ µû¶ó Ã¹¹ß °áÁ€
        {
            if(y>=0)
                temp = -1;
            else
                temp = 1;

            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }

        if(y>=0 && temp==-1) // Ã¹¹ß Á¶°Ç
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
        else if(y<0 && temp==1)
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }

        for(int i=0;i<y_number;i++)
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794+(i+1)*dy;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;

            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794+(i+1)*dy;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }

        if(abs(y_residual)>=0.001)
        {
            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794+y;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;

            temp *= -1;

            // ÂøÁö¹ß °è»ê
            _foot_step(index,0) = x;
            _foot_step(index,1) = -temp*0.127794+y;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
    }


    ///////////////// ÁŠÀÚž® ÈžÀü /////////////////

    if(theta_number!=0 || abs(theta_residual)>= 0.02)
    {
        for (int i =0 ; i<theta_number; i++)
        {
            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin((i+1)*dtheta)+x;
            _foot_step(index,1) = -temp*0.127794*cos((i+1)*dtheta)+y;
            _foot_step(index,5) = (i+1)*dtheta;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }

        if(abs(theta_residual) >= 0.02)
        {
            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin(alpha)+x;
            _foot_step(index,1) = -temp*0.127794*cos(alpha)+y;
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;

            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin(alpha)+x;
            _foot_step(index,1) = -temp*0.127794*cos(alpha)+y;
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
        else
        {
            temp *= -1;

            _foot_step(index,0) = temp*0.127794*sin(alpha)+x;
            _foot_step(index,1) = -temp*0.127794*cos(alpha)+y;
            _foot_step(index,5) = alpha;
            _foot_step(index,6) = 0.5+temp*0.5;
            index++;
        }
    }
}
