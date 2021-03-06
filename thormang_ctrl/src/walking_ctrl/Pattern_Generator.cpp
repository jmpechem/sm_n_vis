#include "Pattern_Generator.h"
#include <vector>
void Pattern_generator::Desired_output(int cnt, int time_parameter_init,int time_parameter_end, Vector6D p_init,Vector6D p_end, Vector6D ori_init, Vector6D ori_end, Vector6D& desired_position,Vector6D& desired_orientation)
{
    desired_position(0) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(0),0.0,p_end(0),0.0);
    desired_position(1) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(1),0.0,p_end(1),0.0);
    desired_position(2) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(2),0.0,p_end(2),0.0);
    desired_position(3) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(3),0.0,p_end(3),0.0);
    desired_position(4) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(4),0.0,p_end(4),0.0);
    desired_position(5) = Cubic(cnt,time_parameter_init,time_parameter_end,p_init(5),0.0,p_end(5),0.0);
    desired_orientation(0) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(0),0.0,ori_end(0),0.0);
    desired_orientation(1) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(1),0.0,ori_end(1),0.0);
    desired_orientation(2) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(2),0.0,ori_end(2),0.0);
    desired_orientation(3) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(3),0.0,ori_end(3),0.0);
    desired_orientation(4) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(4),0.0,ori_end(4),0.0);
    desired_orientation(5) = Cubic(cnt,time_parameter_init,time_parameter_end,ori_init(5),0.0,ori_end(5),0.0);
}

void Pattern_generator::Egress(int t_start, int gap, int gap2, VectorXD t_egress)
{
    VectorXD constant;
    constant.resize(13);
    constant.setZero();

    constant(0) = -0.33;
    constant(1) = 0.39;
    constant(2) = -0.30;
    constant(3) = 0.3;
    constant(4) = 35*DEGREE;
    constant(5) = 0.56;
    constant(6) = 1.2;
    constant(7) = 0.1;
    constant(8) = 0.25;// //comx 0.15 OK/ 0.15 OK/ 0.2 OK / 0.25 X/ 0.25 ?/
    constant(9) = -0.15; //comy -0.1 OK/ -0.15 OK/ -0.1 OK/ -0.1 X/ -0.5 ?/
    constant(10) = 0.35;//+constant(9);//0.45;//lfootz
    constant(11) = 0.3; //lfooty
    constant(12) = 1.3;
    //constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)) =

    Vector6D desired_p;
    desired_p.setZero();
    Vector6D desired_ori;
    desired_ori.setZero();

    /*int sequence = 16;
    VectorXD gap_egress;
    gap_egress.resize(sequence);
    gap_egress.setZero();
    VectorXD t_egress;
    t_egress.resize(sequence+1);
    t_egress(0) = 0;
    for (int i=0; i<sequence; i++)
    {
        gap_egress(i) = gap;
        gap_egress(0) = gap2;
        gap_egress(5) = gap;
        t_egress(i+1) = t_egress(i)+gap_egress(i);
    }*/


    if (_cnt == 0)
    {
        leg_init(0) = -_T_LFoot_global[5].translation()(0) + _T_LFoot_global[0].translation()(0);
        leg_init(1) = -_T_LFoot_global[5].translation()(1) + _T_LFoot_global[0].translation()(1);
        leg_init(2) = -_T_LFoot_global[5].translation()(2) + _T_LFoot_global[0].translation()(2);
        leg_init(3) = -_T_RFoot_global[5].translation()(0) + _T_RFoot_global[0].translation()(0);
        leg_init(4) = -_T_RFoot_global[5].translation()(1) + _T_RFoot_global[0].translation()(1);
        leg_init(5) = -_T_RFoot_global[5].translation()(2) + _T_RFoot_global[0].translation()(2);
        //leg_ori(0) = -_T_LFoot_global_euler[5].rotation()(0) +_T_LFoot_global_euler[0].roataion()(0);
        //leg_ori(1) = -_T_LFoot_global_euler[5].rotation()(1) +_T_LFoot_global_euler[0].roataion()(1);
        //leg_ori(2) = -_T_LFoot_global_euler[5].rotation()(2) +_T_LFoot_global_euler[0].roataion()(2);
        //leg_ori(3) = -_T_LFoot_global_euler[5].rotation()(3) +_T_LFoot_global_euler[0].roataion()(3);
        //leg_ori(4) = -_T_LFoot_global_euler[5].rotation()(4) +_T_LFoot_global_euler[0].roataion()(4);
        //leg_ori(5) = -_T_LFoot_global_euler[5].rotation()(5) +_T_LFoot_global_euler[0].roataion()(5);
        cout<< "leg_init : "<< leg_init<<endl;
    }
            //cout<< "_cnt : "<< _cnt<<endl;

           if(_cnt>=t_egress(0) && _cnt < t_egress(1))
           {
               if(_cnt == t_egress(0))
               {
                   p_parameter_init = leg_init;
                   p_parameter_end << constant(0), 0.0, constant(1), constant(0), 0.0, constant(1);
                   ori_parameter_init << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                   ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
               }
               Desired_output(_cnt, t_egress(0), t_egress(1), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);
               /*
               desired_p(0) = Cubic(_cnt,0,t_start + gap2,leg_init(0),0.0,constant(0),0.0);
               desired_p(1) = Cubic(_cnt,0,t_start + gap2,leg_init(1),0.0,0.0,0.0);
               desired_p(2) = Cubic(_cnt,0,t_start + gap2,leg_init(2),0.0,constant(1),0.0);
               desired_p(3) = Cubic(_cnt,0,t_start + gap2,leg_init(3),0.0,constant(0),0.0);
               desired_p(4) = Cubic(_cnt,0,t_start + gap2,leg_init(4),0.0,0.0,0.0);
               desired_p(5) = Cubic(_cnt,0,t_start + gap2,leg_init(5),0.0,constant(1),0.0);
               desired_ori(0) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0); //z
               desired_ori(1) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0); //y
               desired_ori(2) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0);
               desired_ori(4) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,0,t_start + gap2,0.0,0.0,0.0,0.0);
               */
               //printf("0");
           }
           if(_cnt>=t_egress(1) && _cnt < t_egress(2))
            {
                if (_cnt ==  t_egress(1) )
                {
                    p_parameter_init = p_parameter_end;
                    p_parameter_end << constant(0), 0.0, constant(1), constant(0), 0.0, constant(1);
                    ori_parameter_init = ori_parameter_end;
                    ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                }
                /*desired_p(0) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,constant(0),0.0,constant(0),0.0);
                desired_p(1) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(2) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,constant(1),0.0,constant(1),0.0);
                desired_p(3) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,constant(0),0.0,constant(0),0.0);
                desired_p(4) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(5) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,constant(1),0.0,constant(1),0.0);
                desired_ori(0) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0); //z
                desired_ori(1) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0); //y
                desired_ori(2) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0); //x
                desired_ori(3) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(4) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(5) = Cubic(_cnt,t_start+gap2,t_start + gap+gap2,0.0,0.0,0.0,0.0);
                */
                Desired_output(_cnt, t_egress(1), t_egress(2), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

                //printf("1");

                //cout<< "desird_foot in pattern : "<< desired_p<<endl;

            }

            else if (_cnt>=t_egress(2) && _cnt < t_egress(3))
            {
               if(_cnt == t_egress(2))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0), 0.0, constant(1), constant(2), 0.0, constant(3);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
               }

                /*desired_p(0) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,constant(0),0.0,constant(0),0.0);
                desired_p(1) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(2) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,constant(1),0.0,constant(1),0.0);
                desired_p(3) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,constant(0),0.0,constant(2),0.0);;
                desired_p(4) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(5) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,constant(1),0.0,constant(3),0.0);
                desired_ori(0) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0); //z
                desired_ori(1) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0); //y
                desired_ori(2) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0); //x
                desired_ori(3) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(4) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(5) = Cubic(_cnt,t_start + gap+gap2,t_start + 2*gap+gap2,0.0,0.0,0.0,0.0);*/
                Desired_output(_cnt, t_egress(2), t_egress(3), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

                //printf("2");

            }

            else if (_cnt>=t_egress(3) && _cnt < t_egress(4))
            {
               if(_cnt == t_egress(3))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0), 0.0, constant(1), constant(12)*constant(2), 0.0, constant(3);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << -constant(4), 0.0, 0.0, constant(4), 0.0, 0.0;
               }
                /*desired_p(0) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,constant(0),0.0,constant(0),0.0);
                desired_p(1) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(2) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,constant(1),0.0,constant(1),0.0);
                desired_p(3) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,constant(2),0.0,constant(12)*constant(2),0.0);;
                desired_p(4) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(5) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,constant(3),0.0,constant(3),0.0);
                desired_ori(0) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,-constant(4),0.0); //z
                desired_ori(1) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0); //y
                desired_ori(2) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0); //x
                desired_ori(3) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,constant(4),0.0);
                desired_ori(4) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(5) = Cubic(_cnt,t_start + 2*gap+gap2,t_start + 3*gap+gap2,0.0,0.0,0.0,0.0);*/
                Desired_output(_cnt, t_egress(3), t_egress(4), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

                //printf("3");

            }
            else if (_cnt>=t_egress(4) && _cnt < t_egress(5)) //right foot contact ground
            {
               if(_cnt == t_egress(4))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0), 0.0, constant(1), constant(6)*constant(2), 0.0, constant(5);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << -constant(4), 0.0, 0.0, constant(4), 0.0, 0.0;
               }

                /*desired_p(0) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,constant(0),0.0,constant(0),0.0);
                desired_p(1) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(2) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,constant(1),0.0,constant(1),0.0);
                desired_p(3) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,constant(12)*constant(2),0.0,constant(6)*constant(2),0.0);;
                desired_p(4) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0);
                desired_p(5) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,constant(3),0.0,constant(5),0.0);
                desired_ori(0) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,-constant(4),0.0,-constant(4),0.0); //z
                desired_ori(1) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0); //y
                desired_ori(2) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0); //x
                desired_ori(3) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,constant(4),0.0,constant(4),0.0);
                desired_ori(4) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0);
                desired_ori(5) = Cubic(_cnt,t_start + 3*gap+gap2,t_start + 4*gap+gap2,0.0,0.0,0.0,0.0);*/
                Desired_output(_cnt, t_egress(4), t_egress(5), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

                //printf("4");
            }
           else if (_cnt>=t_egress(5) && _cnt < t_egress(6))
           {
               if(_cnt == t_egress(5))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)), -constant(8)*sin(constant(4))+constant(9)*cos(constant(4)), constant(1)+constant(7), constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)), -constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)), constant(5)+constant(7);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << -constant(4), 0.0, 0.0, constant(4), 0.0, 0.0;
               }
               // c1+c7*cos(c5)+c8*sin(c5), -c7*sin(c5)+c8*cos(c5), c2+0.03, c3+c7*cos(-c5)+c8*sin(-c5), -c7*sin(-c5)+c8*cos(-c5), c6+0.03;
               /*desired_p(0) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,constant(0),0.0,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0);
               desired_p(1) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4)),0.0);
               desired_p(2) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,constant(1),0.0,constant(1)+constant(7),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,constant(6)*constant(2),0.0,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0);
               desired_p(5) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,constant(5),0.0,constant(5)+constant(7),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,-constant(4),0.0,-constant(4),0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,0.0,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,constant(4),0.0,constant(4),0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 4*gap+gap2,t_start + 5*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(5), t_egress(6), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("5");
           }
           else if (_cnt>=t_egress(6) && _cnt < t_egress(7))
           {
               if(_cnt == t_egress(6))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)), -constant(8)*sin(constant(4))+constant(9)*cos(constant(4)), constant(10), constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)), -constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)), constant(5)+constant(7);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << -constant(4), -10*DEGREE, 20*DEGREE, constant(4), 0.0, 0.0;
               }
               /*desired_p(0) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0);
               desired_p(1) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4)),0.0,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4)),0.0);
               desired_p(2) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,constant(1)+constant(7),0.0,constant(10),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0);
               desired_p(5) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,constant(5)+constant(7),0.0,constant(5)+constant(7),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,-constant(4),0.0,-constant(4),0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,0.0,0.0,-10*DEGREE,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,0.0,0.0,20*DEGREE,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,constant(4),0.0,constant(4),0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 5*gap+gap2,t_start + 6*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(6), t_egress(7), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("6");
           }
           else if (_cnt>=t_egress(7) && _cnt < t_egress(8))
           {
               if(_cnt == t_egress(7))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)), -constant(8)*sin(constant(4))+constant(9)*cos(constant(4))+constant(11), constant(10), constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)), -constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)), constant(5)+constant(7);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << -constant(4), -10*DEGREE, 20*DEGREE, constant(4), 0.0, 0.0;
               }

               /*desired_p(0) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0);
               desired_p(1) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4)),0.0,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4))+constant(11),0.0);
               desired_p(2) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,constant(10),0.0,constant(10),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0);
               desired_p(5) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,constant(5)+constant(7),0.0,constant(5)+constant(7),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,-constant(4),0.0,-constant(4),0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,-10*DEGREE,0.0,-10*DEGREE,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,20*DEGREE,0.0,-20*DEGREE,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,constant(4),0.0,constant(4),0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 6*gap+gap2,t_start + 7*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(7), t_egress(8), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("7");
           }
           else if (_cnt>=t_egress(8) && _cnt < t_egress(9))
           {
               if(_cnt == t_egress(8))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)), 0.0, constant(10), constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)), 0.0, constant(5)+constant(7);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << 0.0, -5*DEGREE, 0.0, 0.0, 0.0, 0.0;
               }

               /*desired_p(0) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0);
               desired_p(1) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,-constant(8)*sin(constant(4))+constant(9)*cos(constant(4))+constant(11),0.0,0.0,0.0);
               desired_p(2) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,constant(10),0.0,constant(10),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,-constant(8)*sin(-constant(4))+constant(9)*cos(-constant(4)),0.0,0.0,0.0);
               desired_p(5) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,constant(5)+constant(7),0.0,constant(5)+constant(7),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,-constant(4),0.0,0.0,0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,-10*DEGREE,0.0,-5*DEGREE,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,-20*DEGREE,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,constant(4),0.0,0.0,0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 7*gap+gap2,t_start + 8*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(8), t_egress(9), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("8");
           }
           else if (_cnt>=t_egress(9) && _cnt < t_egress(10))
           {
               if(_cnt == t_egress(9))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)), 0.0, constant(5)+constant(7), 0.0, 0.0, constant(5)+constant(7);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
               }

               /*desired_p(0) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0);
               desired_p(1) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(2) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,constant(10),0.0,constant(5)+0.1,0.0);
               desired_p(3) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,constant(6)*constant(2)+constant(8)*cos(-constant(4))+constant(9)*sin(-constant(4)),0.0,0.0,0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(5) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,constant(5)+constant(7),0.0,constant(5)+constant(7),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,-5*DEGREE,0.0,0.0,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 8*gap+gap2,t_start + 9*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(9), t_egress(10), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("9");
           }
           else if (_cnt>=t_egress(10) && _cnt < t_egress(11))
           {
               if(_cnt == t_egress(10))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << (constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2, 0.0, constant(5), -(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2, 0.0, constant(5);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
               }

               /*desired_p(0) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)),0.0,(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0);
               desired_p(1) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(2) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,constant(5)+constant(7),0.0,constant(5),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,-(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(5) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,constant(5)+constant(7),0.0,constant(5),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 9*gap+gap2,t_start + 10*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(10), t_egress(11), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("10");
           }
           else if (_cnt >= t_egress(11))
           {
               if(_cnt == t_egress(11))
               {
                   p_parameter_init = p_parameter_end;
                   p_parameter_end << (constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2, 0.0, constant(5), -(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2, 0.0, constant(5);
                   ori_parameter_init = ori_parameter_end;
                   ori_parameter_end << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
               }

               /*desired_p(0) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0,(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0);
               desired_p(1) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(2) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,constant(5),0.0,constant(5),0.0);
               desired_p(3) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,-(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0,-(constant(0)+constant(8)*cos(constant(4))+constant(9)*sin(constant(4)))/2,0.0);;
               desired_p(4) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0);
               desired_p(5) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,constant(5),0.0,constant(5),0.0);
               desired_ori(0) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0); //z
               desired_ori(1) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0); //y
               desired_ori(2) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0); //x
               desired_ori(3) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(4) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0);
               desired_ori(5) = Cubic(_cnt,t_start + 10*gap+gap2,t_start + 11*gap+gap2,0.0,0.0,0.0,0.0);*/
               Desired_output(_cnt, t_egress(11), t_egress(12), p_parameter_init, p_parameter_end, ori_parameter_init, ori_parameter_end, desired_p, desired_ori);

               //printf("11");
           }

           Foot_trajectory.LFoot.translation()(0) = desired_p(0);
           Foot_trajectory.LFoot.translation()(1) = desired_p(1);
           Foot_trajectory.LFoot.translation()(2) = desired_p(2);

           Foot_trajectory.RFoot.translation()(0) = desired_p(3);
           Foot_trajectory.RFoot.translation()(1) = desired_p(4);
           Foot_trajectory.RFoot.translation()(2) = desired_p(5);

           for(int i=0;i<3;i++){
               _T_LFoot_global_euler(i) = desired_ori(i);
               _T_RFoot_global_euler(i) = desired_ori(i+3);
           }
}

void Pattern_generator::Foot_swing_trajectory(int cnt, int t_start, int interval, Vector3D& target_p, Vector3D current_p, Vector3D desired_p, double height)
{
    //target_p.resize(3);
    //target_p.zeros();
    double foot_speed = 0.0;


    if (cnt < t_start+interval/4)
    //if (cnt >= t_start && cnt < t_start+interval/4)
    {
        target_p(0) = Cubic(cnt,t_start,t_start+interval/4,current_p(0),0.0,current_p(0),0.0);
        target_p(1) = Cubic(cnt,t_start,t_start+interval/4,current_p(1),0.0,current_p(1),0.0);
        target_p(2) = Cubic(cnt,t_start,t_start+interval/4,current_p(2),0.0,current_p(2)+height,foot_speed);
    }

    else if (cnt >= t_start+interval/4 && cnt < t_start+interval*3/4)
    {
        cout<<"yaho"<<_cnt<<endl;
        target_p(0) = Cubic(cnt,t_start+interval/4,t_start+interval*3/4,current_p(0),0.0,current_p(0)+desired_p(0),0.0);
        target_p(1) = Cubic(cnt,t_start+interval/4,t_start+interval*3/4,current_p(1),0.0,current_p(1)+desired_p(1),0.0);
        target_p(2) = Cubic(cnt,t_start+interval/4,t_start+interval*3/4,current_p(2)+height,foot_speed,current_p(2)+height+desired_p(2),-foot_speed);
    }

    else if (cnt >= t_start+interval*3/4)
    //else if (cnt >= t_start+interval*3/4 && cnt < t_start+interval)
    {
        target_p(0) = Cubic(cnt,t_start+interval*3/4,t_start+interval,current_p(0)+desired_p(0),0.0,current_p(0)+desired_p(0),0.0);
        target_p(1) = Cubic(cnt,t_start+interval*3/4,t_start+interval,current_p(1)+desired_p(1),0.0,current_p(1)+desired_p(1),0.0);
        target_p(2) = Cubic(cnt,t_start+interval*3/4,t_start+interval,current_p(2)+height+desired_p(2),-foot_speed,current_p(2)+desired_p(2),0.0);
    }


}


void Pattern_generator::Foot_trajectory_update2()
{
    Vector3D target = _Target_data;
    double height = 0.05;

    Vector3D _init_RFoot;
    _init_RFoot = _init_info._XR_support_init.translation();

    //cout << "init" << _init_RFoot << endl;
    //cout << "ttt" <<target << endl;
    Vector3D RFoot_output;

    cout << "cnt" << _cnt << endl;
    //cout << "start_t" << _T_Start_real+_T_Double1 << endl;
    //cout << "duiraion " << _T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2 << endl;
    Foot_swing_trajectory(_cnt,_T_Start_real+_T_Double1,_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2,RFoot_output, _init_RFoot,target,height);
    Foot_trajectory.LFoot.translation() = _init_info._XL_support_init.translation();
    Foot_trajectory.RFoot.translation() = RFoot_output;
 cout << "tttaaaa" <<RFoot_output << endl;
    Foot_trajectory.LFoot.linear() = _init_info._XL_support_init.linear();

    Foot_trajectory.RFoot.linear() = _init_info._XR_support_init.linear();

}

void Pattern_generator::Heel_Toe_Motion_pattern()
{
    double L_ankle_angle, R_ankle_angle;
    L_ankle_angle = 0.0;
    R_ankle_angle = 0.0;

    Vector3D Front_init_angkle_edge; // �߹ٴ� ��������� �ٶ��� ankle�� �ġ
    Front_init_angkle_edge(0) = -0.13;
    Front_init_angkle_edge(1) = 0.0;
    Front_init_angkle_edge(2) = 0.092;

    Vector3D Rear_init_ankle_edge;
    Rear_init_ankle_edge(0) = 0.13;
    Rear_init_ankle_edge(1) = 0.0;
    Rear_init_ankle_edge(2) = 0.092;


    if(_foot_step(_step_number,6) == 0) //left swing
    {
        if(_cnt < _T_Last-2*_T_Double2)
            L_ankle_angle = Cubic(_cnt,_T_Start+_T_Total/2.0,_T_Last-2*_T_Double2,0.0,0.0,-20*DEGREE,0.0);
        else
            L_ankle_angle = Cubic(_cnt,_T_Last-1.5*_T_Double2,_T_Last,-20*DEGREE,0.0,0.0,0.0);
        R_ankle_angle = 0.0;
    }
    else
    {
        if(_cnt < _T_Last-2*_T_Double2)
            R_ankle_angle = Cubic(_cnt,_T_Start+_T_Total/2.0,_T_Last-2*_T_Double2,0.0,0.0,-20*DEGREE,0.0);
        else
            R_ankle_angle = Cubic(_cnt,_T_Last-1.5*_T_Double2,_T_Last,-20*DEGREE,0.0,0.0,0.0);
        L_ankle_angle = 0.0;
    }

    Foot_trajectory_global.LFoot_euler(1) = Foot_trajectory_global.LFoot_euler(1)+L_ankle_angle;
    Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));
    Vector3D temp;
    temp = Rotate_with_Y(L_ankle_angle)*Rear_init_ankle_edge;
    temp = temp-Rear_init_ankle_edge;
    Foot_trajectory_global.LFoot.translation() = Foot_trajectory_global.LFoot.translation()+temp;

    Foot_trajectory_global.RFoot_euler(1) = Foot_trajectory_global.RFoot_euler(1)+R_ankle_angle;
    Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));

    temp = Rotate_with_Y(R_ankle_angle)*Rear_init_ankle_edge;
    temp = temp-Rear_init_ankle_edge;
    Foot_trajectory_global.RFoot.translation() = Foot_trajectory_global.RFoot.translation()+temp;
}



void Pattern_generator::Heel_Toe_Motion()
{
    double L_ankle_angle, R_ankle_angle;
    L_ankle_angle = 0.0;
    R_ankle_angle = 0.0;

    double Leg_max = 0.595; // �ٸ� ���ʶ� ����

    Vector3D Front_init_angkle_edge; // �߹ٴ� ��������� �ٶ��� ankle�� �ġ
    Front_init_angkle_edge(0) = -0.13;
    Front_init_angkle_edge(1) = 0.0;
    Front_init_angkle_edge(2) = 0.092;

    Vector3D Rear_init_ankle_edge;
    Rear_init_ankle_edge(0) = 0.13;
    Rear_init_ankle_edge(1) = 0.0;
    Rear_init_ankle_edge(2) = 0.092;

    double L_leg_length, R_leg_length;

    Vector3D L_Hip_Ankle, R_Hip_Ankle;
    L_Hip_Ankle = Foot_trajectory_global.LFoot.translation() - _T_LFoot_global[0].translation();
    R_Hip_Ankle = Foot_trajectory_global.RFoot.translation() - _T_RFoot_global[0].translation();

    L_leg_length = pow(L_Hip_Ankle(0),2.0)+pow(L_Hip_Ankle(1),2.0)+pow(L_Hip_Ankle(2),2.0);
    R_leg_length = pow(R_Hip_Ankle(0),2.0)+pow(R_Hip_Ankle(1),2.0)+pow(R_Hip_Ankle(2),2.0);

    if(L_leg_length >= (Leg_max*Leg_max))
    {
        if(Foot_trajectory_global.LFoot.translation()(0) < 0.0) // ���� ���ʿ� �ִ� ��Ȳ �߸��� ������ ���ƾߴ�
        {

            cout<< "LHeelToe ON1111!" << endl;
            double a, b, c;
            a = -2*pow(Front_init_angkle_edge(2),2.0)-2*pow(Front_init_angkle_edge(0),2.0)+2*L_Hip_Ankle(0)*Front_init_angkle_edge(0)+2*L_Hip_Ankle(2)*Front_init_angkle_edge(2);
            b = 2*L_Hip_Ankle(0)*Front_init_angkle_edge(2)-2*L_Hip_Ankle(2)*Front_init_angkle_edge(0);
            c = pow(Leg_max,2.0)-L_leg_length-2*pow(Front_init_angkle_edge(0),2.0)-2*pow(Front_init_angkle_edge(2),2.0)+2*L_Hip_Ankle(0)*Front_init_angkle_edge(0)+2*L_Hip_Ankle(2)*Front_init_angkle_edge(2);

            double d;
            d = sqrt(pow(a,2.0)+pow(b,2.0));

            double x, x_plus_theta;
            x = asin(a/d);
            x_plus_theta = asin(c/d);

            Vector3D Theta_temp;

            Theta_temp(0) = -(x_plus_theta-x); // ����� �������� Toe ankle�� ankle orientation� ��ȣ�� �ݴ���!
            Theta_temp(1) = -(x_plus_theta+180*DEGREE-x);
            Theta_temp(2) = -(x_plus_theta-x-180*DEGREE);

            for(int i=0; i<3; i++)
            {
                if(Theta_temp(i) > 0.0 && Theta_temp(i) < 45*DEGREE)
                    L_ankle_angle = Theta_temp(i);
            }

            Foot_trajectory_global.LFoot_euler(1) = Foot_trajectory_global.LFoot_euler(1)+L_ankle_angle;
            Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));
            cout << "Theta_temp" << Theta_temp << endl;
            cout << "L_ankle_angle" << L_ankle_angle << endl;
            Vector3D temp;
            temp = Rotate_with_Y(L_ankle_angle)*Front_init_angkle_edge;
            temp = temp-Front_init_angkle_edge;
            Foot_trajectory_global.LFoot.translation() = Foot_trajectory_global.LFoot.translation()+temp;



        }
        else
        {
            cout<< "LHeelToe ON!" << endl;
            double a, b, c;
            a = -2*pow(Rear_init_ankle_edge(2),2.0)-2*pow(Rear_init_ankle_edge(0),2.0)+2*L_Hip_Ankle(0)*Rear_init_ankle_edge(0)+2*L_Hip_Ankle(2)*Rear_init_ankle_edge(2);
            b = 2*L_Hip_Ankle(0)*Rear_init_ankle_edge(2)-2*L_Hip_Ankle(2)*Rear_init_ankle_edge(0);
            c = pow(Leg_max,2.0)-L_leg_length-2*pow(Rear_init_ankle_edge(0),2.0)-2*pow(Rear_init_ankle_edge(2),2.0)+2*L_Hip_Ankle(0)*Rear_init_ankle_edge(0)+2*L_Hip_Ankle(2)*Rear_init_ankle_edge(2);

            double d;
            d = sqrt(pow(a,2.0)+pow(b,2.0));


            double x, x_plus_theta;
            x = asin(a/d);
            x_plus_theta = asin(c/d);

            Vector3D Theta_temp;

            Theta_temp(0) = -(x_plus_theta-x); // ����� �������� Toe ankle�� ankle orientation� ��ȣ�� �ݴ���!
            Theta_temp(1) = -(x_plus_theta+180*DEGREE-x);
            Theta_temp(2) = -(x_plus_theta-x-180*DEGREE);

            for(int i=0; i<3; i++)
            {
                if(Theta_temp(i) < 0.0 && Theta_temp(i) > -45*DEGREE)
                    L_ankle_angle = Theta_temp(i);
            }

            Foot_trajectory_global.LFoot_euler(1) = Foot_trajectory_global.LFoot_euler(1)+L_ankle_angle;
            Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));

            Vector3D temp;
            temp = Rotate_with_Y(L_ankle_angle)*Rear_init_ankle_edge;
            temp = temp-Rear_init_ankle_edge;
            Foot_trajectory_global.LFoot.translation() = Foot_trajectory_global.LFoot.translation()+temp;

        }
    }

    if(R_leg_length >= (Leg_max*Leg_max))
    {
        if(Foot_trajectory_global.RFoot.translation()(0) < 0.0) // ���� ���ʿ� �ִ� ��Ȳ �߸��� ������ ���ƾߴ�
        {
            cout<< "LHeelToe ON!" << endl;
            double a, b, c;
            a = -2*pow(Front_init_angkle_edge(2),2.0)-2*pow(Front_init_angkle_edge(0),2.0)+2*R_Hip_Ankle(0)*Front_init_angkle_edge(0)+2*R_Hip_Ankle(2)*Front_init_angkle_edge(2);
            b = 2*R_Hip_Ankle(0)*Front_init_angkle_edge(2)-2*R_Hip_Ankle(2)*Front_init_angkle_edge(0);
            c = pow(Leg_max,2.0)-R_leg_length-2*pow(Front_init_angkle_edge(0),2.0)-2*pow(Front_init_angkle_edge(2),2.0)+2*R_Hip_Ankle(0)*Front_init_angkle_edge(0)+2*R_Hip_Ankle(2)*Front_init_angkle_edge(2);

            double d;
            d = sqrt(pow(a,2.0)+pow(b,2.0));


            double x, x_plus_theta;
            x = asin(a/d);
            x_plus_theta = asin(c/d);

            Vector3D Theta_temp;

            Theta_temp(0) = -(x_plus_theta-x); // ����� �������� Toe ankle�� ankle orientation� ��ȣ�� �ݴ���!
            Theta_temp(1) = -(x_plus_theta+180*DEGREE-x);
            Theta_temp(2) = -(x_plus_theta-x-180*DEGREE);

            for(int i=0; i<3; i++)
            {
                if(Theta_temp(i) > 0.0 && Theta_temp(i) < 45*DEGREE)
                    R_ankle_angle = Theta_temp(i);
            }

            Foot_trajectory_global.RFoot_euler(1) = Foot_trajectory_global.RFoot_euler(1)+R_ankle_angle;
            Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));

            Vector3D temp;
            temp = Rotate_with_Y(R_ankle_angle)*Front_init_angkle_edge;
            temp = temp-Front_init_angkle_edge;
            Foot_trajectory_global.RFoot.translation() = Foot_trajectory_global.RFoot.translation()+temp;

        }
        else
        {
            cout<< "RHeelToe ON1111!" << endl;
            double a, b, c;
            a = -2*pow(Rear_init_ankle_edge(2),2.0)-2*pow(Rear_init_ankle_edge(0),2.0)+2*R_Hip_Ankle(0)*Rear_init_ankle_edge(0)+2*R_Hip_Ankle(2)*Rear_init_ankle_edge(2);
            b = 2*R_Hip_Ankle(0)*Rear_init_ankle_edge(2)-2*R_Hip_Ankle(2)*Rear_init_ankle_edge(0);
            c = pow(Leg_max,2.0)-R_leg_length-2*pow(Rear_init_ankle_edge(0),2.0)-2*pow(Rear_init_ankle_edge(2),2.0)+2*R_Hip_Ankle(0)*Rear_init_ankle_edge(0)+2*R_Hip_Ankle(2)*Rear_init_ankle_edge(2);

            double d;
            d = sqrt(pow(a,2.0)+pow(b,2.0));


            double x, x_plus_theta;
            x = asin(a/d);
            x_plus_theta = asin(c/d);

            Vector3D Theta_temp;

            Theta_temp(0) = (x_plus_theta-x); // ������ ankle-contact point���͸� �Ȱ��� �Ἥ ��ȣ �ݴ�
            Theta_temp(1) = (x_plus_theta+180*DEGREE-x);
            Theta_temp(2) = (x_plus_theta-x-180*DEGREE);

            for(int i=0; i<3; i++)
            {
                if(Theta_temp(i) < 0.0 && Theta_temp(i) > -45*DEGREE)
                    R_ankle_angle = Theta_temp(i);
            }
            Foot_trajectory_global.RFoot_euler(1) = Foot_trajectory_global.RFoot_euler(1)+R_ankle_angle;
            Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));

            Vector3D temp;
            temp = Rotate_with_Y(R_ankle_angle)*Rear_init_ankle_edge;
            temp = temp-Rear_init_ankle_edge;
            Foot_trajectory_global.RFoot.translation() = Foot_trajectory_global.RFoot.translation()+temp;
            cout << "Theta_temp" << Theta_temp << endl;
            cout << "R_ankle_angle" << R_ankle_angle <<  endl;

            R_Hip_Ankle = Foot_trajectory_global.RFoot.translation() - _T_RFoot_global[0].translation();
            cout << "RHipankle" << R_Hip_Ankle << endl;
        }
    }

    //L_Hip_Ankle = Foot_trajectory_global.LFoot.translation() - _T_LFoot_global[0].translation();
    //cout << "LHipankle" << L_Hip_Ankle << endl;
    //R_Hip_Ankle = Foot_trajectory_global.RFoot.translation() - _T_RFoot_global[0].translation();
    //cout << "RHipankle" << R_Hip_Ankle << endl;


    //fprintf(fp5,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",Foot_trajectory_global.RFoot.translation()(0),Foot_trajectory_global.RFoot.translation()(1),Foot_trajectory_global.RFoot.translation()(2),Foot_trajectory_global.LFoot.translation()(0),Foot_trajectory_global.LFoot.translation()(1),Foot_trajectory_global.LFoot.translation()(2),R_ankle_angle,L_ankle_angle,L_Hip_Ankle(0),L_Hip_Ankle(1),L_Hip_Ankle(2));


    /*Foot_trajectory_global.LFoot_euler(1) = Foot_trajectory_global.LFoot_euler(1)+L_ankle_angle;
    Foot_trajectory_global.RFoot_euler(1) = Foot_trajectory_global.RFoot_euler(1)+R_ankle_angle;
    Foot_trajectory_global.LFoot.linear() = Rotate_with_Z(Foot_trajectory_global.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.LFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.LFoot_euler(0));
    Foot_trajectory_global.RFoot.linear() = Rotate_with_Z(Foot_trajectory_global.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory_global.RFoot_euler(1))*Rotate_with_X(Foot_trajectory_global.RFoot_euler(0));


    Vector3D temp;
    temp = Rotate_with_Y(L_ankle_angle)*init_angkle_edge;
    temp = temp-init_angkle_edge;
    Foot_trajectory_global.LFoot.translation() = Foot_trajectory_global.LFoot.translation()+temp;
    temp = Rotate_with_Y(R_ankle_angle)*init_angkle_edge;
    temp = temp-init_angkle_edge;
    Foot_trajectory_global.RFoot.translation() = Foot_trajectory_global.RFoot.translation()+temp;
*/
    //Vector3D L_Hip_Ankle, R_Hip_Ankle;
    /*L_Hip_Ankle = Foot_trajectory_global.LFoot.translation() - _T_LFoot_global[0].translation();
    R_Hip_Ankle = Foot_trajectory_global.RFoot.translation() - _T_RFoot_global[0].translation();

    cout << "LHipankle" << L_Hip_Ankle << endl;
    cout << "RHipankle" << R_Hip_Ankle << endl;*/
}
void Pattern_generator::Change_Local_Pattern()
{
    HTransform reference;

    if(_foot_step(_step_number,6) == 1)
    {
        reference.translation() = Foot_trajectory_global.LFoot.translation();
        reference.linear() = Foot_trajectory_global.LFoot.linear();
    }
    else
    {
        reference.translation() = Foot_trajectory_global.RFoot.translation();
        reference.linear() = Foot_trajectory_global.RFoot.linear();
    }


    if(_gyro_frame_flag == true)
    {
        GlobalGyroframe(Trunk_trajectory_global,reference,Trunk_trajectory);
        GlobalGyroframe(Foot_trajectory_global.LFoot,reference,Foot_trajectory.LFoot);
        GlobalGyroframe(Foot_trajectory_global.RFoot,reference,Foot_trajectory.RFoot);
        Rot2euler(Foot_trajectory.LFoot.linear(),Foot_trajectory.LFoot_euler);
        Rot2euler(Foot_trajectory.RFoot.linear(),Foot_trajectory.RFoot_euler);
    }
    else
    {
        Globalframe(Trunk_trajectory_global,reference,Trunk_trajectory);
        Globalframe(Foot_trajectory_global.LFoot,reference,Foot_trajectory.LFoot);
        Globalframe(Foot_trajectory_global.RFoot,reference,Foot_trajectory.RFoot);
        Rot2euler(Foot_trajectory.LFoot.linear(),Foot_trajectory.LFoot_euler);
        Rot2euler(Foot_trajectory.RFoot.linear(),Foot_trajectory.RFoot_euler);
    }
}


void Pattern_generator::Change_Global_Pattern()
{
    HTransform reference;

    reference.translation() = Trunk_trajectory.translation();
    reference.linear() = Trunk_trajectory.linear();

    if(_gyro_frame_flag == true)
    {
        GlobalGyroframe(Trunk_trajectory,reference,Trunk_trajectory_global);
        GlobalGyroframe(Foot_trajectory.LFoot,reference,Foot_trajectory_global.LFoot);
        GlobalGyroframe(Foot_trajectory.RFoot,reference,Foot_trajectory_global.RFoot);
        Rot2euler(Foot_trajectory_global.LFoot.linear(),Foot_trajectory_global.LFoot_euler);
        Rot2euler(Foot_trajectory_global.RFoot.linear(),Foot_trajectory_global.RFoot_euler);
    }
    else
    {
        Globalframe(Trunk_trajectory,reference,Trunk_trajectory_global);
        Globalframe(Foot_trajectory.LFoot,reference,Foot_trajectory_global.LFoot);
        Globalframe(Foot_trajectory.RFoot,reference,Foot_trajectory_global.RFoot);
        Rot2euler(Foot_trajectory_global.LFoot.linear(),Foot_trajectory_global.LFoot_euler);
        Rot2euler(Foot_trajectory_global.RFoot.linear(),Foot_trajectory_global.RFoot_euler);
    }
}


void Pattern_generator::Trunk_trajectory_update()
{
    double z_rot = _foot_step_localframe(_step_number,5);

    //Trunk Position
    if(_com_control_flag == true)
    {
        double kp = 1.0;

       // kp = Cubic(abs(_COM_desired(0)-_COM_real_support(0)),0.0,0.05,1.0,0.0,3.0,0.0);
        Trunk_trajectory.translation()(0) = _T_Trunk_support.translation()(0) + kp*(_COM_desired(0) - _COM_real_support(0));
       // kp = Cubic(abs(_COM_desired(1)-_COM_real_support(1)),0.0,0.05,1.0,0.0,3.0,0.0);
        Trunk_trajectory.translation()(1) = _T_Trunk_support.translation()(1) + kp*(_COM_desired(1) - _COM_real_support(1));
       // kp = Cubic(abs(_COM_desired(2)-_COM_real_support(2)),0.0,0.05,1.0,0.0,3.0,0.0);
        Trunk_trajectory.translation()(2) = _COM_desired(2);//_T_Trunk_support.translation()(2) + kp*(_COM_desired(2) - _COM_real_support(2));
    }
    else
    {
double kp = 3.0;
double d = 0.5;
       // Trunk_trajectory.translation()(0) = _COM_desired(0) - _COM_offset(0);

	if(_cnt >= _T_Start && _cnt < _T_Start+0.3*Hz)
	{
		kp = 0+ 3.0*(_cnt-_T_Start)/(0.3*Hz);
		d = 0+ 0.5*(_cnt-_T_Start)/(0.3*Hz);
	}


	if(_cnt == 0) 
		COM_prev = _COM_real_support;			

	if(_cnt ==0)
		COM_pd = 0.0;
 	//Trunk_trajectory.translation()(0) = _T_Trunk_support.translation()(0)+kp*(_COM_desired(0) - _COM_real_support(0) + 0.06) + d*_xd(1)- d*(_COM_real_support(0)-COM_prev(0))/Hz  ;


	double offset_x = 0.0;
	if(_foot_step(_step_number,6) == 1)
	{
		double temp_time = 0.1*Hz;
		if(_cnt < _T_Start_real)
			offset_x = Cubic(_cnt,_T_Start+temp_time,_T_Start_real-temp_time,0.0,0.0,0.02,0.0);
		else
			offset_x = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last+temp_time,_T_Start+_T_Total-temp_time,0.02,0.0,0.0,0.0);
	}

    Trunk_trajectory.translation()(0) = _COM_desired(0) - _COM_offset(0);// + offset_x;// + kp * (_COM_desired(0) - _COM_real_support(0) + 0.06)+(3.0-kp)*COM_pd;
        Trunk_trajectory.translation()(1) = _COM_desired(1) - _COM_offset(1);
        Trunk_trajectory.translation()(2) = _COM_desired(2);

	
	if(_cnt == _T_Start+_T_Total)
		COM_pd = (_COM_desired(0) - _COM_real_support(0) + 0.06);

	COM_prev = _COM_real_support;	


        
	double dt = 1.0/Hz;
	kp = 100.0;
	d = 2000.0;

	//COM_pd = (kp*_COM_desired(0)+d*dt*COM_prev(0))/(kp+d*dt);
	

    }

    //Trunk orientation
    Vector3D Trunk_trajectory_euler;

    if(_cnt < _T_Start_real+_T_Double1)
    {
        for(int i=0; i<2; i++)
            Trunk_trajectory_euler(i) = Cubic(_cnt,_T_Start,_T_Start_real+_T_Double1,_init_COM._Trunk_euler(i),0.0,0.0,0.0);;
        Trunk_trajectory_euler(2) = _init_COM._Trunk_euler(2);
    }

    else if(_cnt >= _T_Start_real+_T_Double1 && _cnt < _T_Start+_T_Total-_T_Double2-_T_rest_last)
    {
        for(int i=0; i<2; i++)
            Trunk_trajectory_euler(i) = 0.0;

        if(_foot_step(_step_number,6) == 2)
            Trunk_trajectory_euler(2) = _init_COM._Trunk_euler(2);
        else
            Trunk_trajectory_euler(2) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_Double2-_T_rest_last,_init_COM._Trunk_euler(2),0.0,z_rot/2.0,0.0);
    }
    else
    {
        for(int i=0; i<2; i++)
            Trunk_trajectory_euler(i) = 0.0;

        if(_foot_step(_step_number,6) == 2)
            Trunk_trajectory_euler(2) = _init_COM._Trunk_euler(2);
        else
            Trunk_trajectory_euler(2) = z_rot/2.0;
    }

    Trunk_trajectory.linear() = Rotate_with_Z(Trunk_trajectory_euler(2))*Rotate_with_Y(Trunk_trajectory_euler(1))*Rotate_with_X(Trunk_trajectory_euler(0));
}

void Pattern_generator::Foot_trajectory_update()
{
    Vector6D target_swing_foot;

    for(int i=0; i<6; i++)
        target_swing_foot(i) = _foot_step_localframe(_step_number,i);

    if(_cnt == 0)
    {
        cout << "target_swing_foot" << target_swing_foot(0) << endl;
    }


    if(_cnt < _T_Start_real+_T_Double1)
    {
        Foot_trajectory.LFoot.translation() = _init_Foot_trajectory.LFoot.translation();
        Foot_trajectory.LFoot_dot.setZero();
        Foot_trajectory.LFoot.translation()(2) = Cubic(_cnt,_T_Start,_T_Start_real,_init_Foot_trajectory.LFoot.translation()(2),0.0,0.0,0.0);

        Foot_trajectory.LFoot_euler = _init_Foot_trajectory.LFoot_euler;

        for(int i=0; i<2; i++)
            Foot_trajectory.LFoot_euler(i) = Cubic(_cnt,_T_Start,_T_Start_real,_init_Foot_trajectory.LFoot_euler(i),0.0,0.0,0.0);

        Foot_trajectory.LFoot.linear() = Rotate_with_Z(Foot_trajectory.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory.LFoot_euler(1))*Rotate_with_X(Foot_trajectory.LFoot_euler(0));

   
        Foot_trajectory.RFoot.translation() = _init_Foot_trajectory.RFoot.translation();
    	Foot_trajectory.RFoot_dot.setZero();
        Foot_trajectory.RFoot.translation()(2) = Cubic(_cnt,_T_Start,_T_Start_real,_init_Foot_trajectory.RFoot.translation()(2),0.0,0.0,0.0);	

        Foot_trajectory.RFoot_euler = _init_Foot_trajectory.RFoot_euler;
        for(int i=0; i<2; i++)
            Foot_trajectory.RFoot_euler(i) = Cubic(_cnt,_T_Start,_T_Start_real,_init_Foot_trajectory.RFoot_euler(i),0.0,0.0,0.0);
        
        Foot_trajectory.RFoot.linear() = Rotate_with_Z(Foot_trajectory.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory.RFoot_euler(1))*Rotate_with_X(Foot_trajectory.RFoot_euler(0));	
       
    }
    else if(_cnt >= _T_Start_real+_T_Double1 && _cnt < _T_Start+_T_Total-_T_Double2-_T_rest_last)
    {
        double t_rest_temp = 0.1*Hz;

        if(_foot_step(_step_number,6) == 1) // �޹� ����
        {
            Foot_trajectory.LFoot.translation() = _init_Foot_trajectory.LFoot.translation();
            Foot_trajectory.LFoot.translation()(2) = 0.0;	
            //Foot_trajectory.LFoot.linear() = _init_Foot_trajectory.LFoot.linear();
            Foot_trajectory.LFoot_euler = _init_Foot_trajectory.LFoot_euler;
            Foot_trajectory.LFoot_euler.setZero();
     
            Foot_trajectory.LFoot_dot.setZero();
            Foot_trajectory.LFoot.linear() = Rotate_with_Z(Foot_trajectory.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory.LFoot_euler(1))*Rotate_with_X(Foot_trajectory.LFoot_euler(0));

            double ankle_temp;
            ankle_temp = -3*DEGREE;
            if(_cnt < _T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0)
            {

                Foot_trajectory.RFoot.translation()(2) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,_ho,0.0);
                Foot_trajectory.RFoot_dot(2) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,_ho,0.0,Hz);

                Foot_trajectory.RFoot_euler(1) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,ankle_temp,0.0);
                Foot_trajectory.RFoot_dot(4) = Cubic_dot(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,ankle_temp,0.0,Hz);
            }
            else
            {
                Foot_trajectory.RFoot_euler(1) = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-t_rest_temp,_T_Start+_T_Total-_T_rest_last,ankle_temp,0.0,0.0,0.0);
                Foot_trajectory.RFoot_dot(4) = Cubic_dot(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-t_rest_temp,_T_Start+_T_Total-_T_rest_last,ankle_temp,0.0,0.0,0.0,Hz);



                Foot_trajectory.RFoot.translation()(2) = Cubic(_cnt,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_ho,0.0,target_swing_foot(2),0.0);
                Foot_trajectory.RFoot_dot(2) = Cubic_dot(_cnt,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_ho,0.0,target_swing_foot(2),0.0,Hz);
            }

            for(int i=0; i<2; i++)
            {
                Foot_trajectory.RFoot_euler(0) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(0+3),0.0);
                Foot_trajectory.RFoot_dot(0+3) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(0+3),0.0,Hz);

                Foot_trajectory.RFoot.translation()(i) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp+0.1*Hz,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.RFoot.translation()(i),0.0,target_swing_foot(i),0.0);
                Foot_trajectory.RFoot_dot(i) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.RFoot.translation()(i),0.0,target_swing_foot(i),0.0,Hz);
            }

          //  for(int i=0; i<2; i++)
          //  {
          //      Foot_trajectory.RFoot_euler(i) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(i+3),0.0);
         //       Foot_trajectory.RFoot_dot(i+3) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(i+3),0.0,Hz);
         //   }
       Foot_trajectory.RFoot_euler(2) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.RFoot_euler(2),0.0,target_swing_foot(5),0.0);
       Foot_trajectory.RFoot_dot(5) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.RFoot_euler(2),0.0,target_swing_foot(5),0.0,Hz);


            Foot_trajectory.RFoot.linear() = Rotate_with_Z(Foot_trajectory.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory.RFoot_euler(1))*Rotate_with_X(Foot_trajectory.RFoot_euler(0));
        }
        else if(_foot_step(_step_number,6) == 0) // �޹� ����
        {
            Foot_trajectory.RFoot.translation() = _init_Foot_trajectory.RFoot.translation();
	        Foot_trajectory.RFoot.translation()(2) = 0.0;
           //Foot_trajectory.RFoot.linear() = _init_Foot_trajectory.RFoot.linear();
            Foot_trajectory.RFoot_euler = _init_Foot_trajectory.RFoot_euler;
            Foot_trajectory.RFoot_euler(0) = 0.0;
	        Foot_trajectory.RFoot_euler(1) = 0.0;
            Foot_trajectory.RFoot_dot.setZero();

            double ankle_temp;
            ankle_temp = -3*DEGREE;

	    Foot_trajectory.RFoot.linear() = Rotate_with_Z(Foot_trajectory.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory.RFoot_euler(1))*Rotate_with_X(Foot_trajectory.RFoot_euler(0));

            if(_cnt < _T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0)
            {

                Foot_trajectory.LFoot.translation()(2) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,_ho,0.0);
                Foot_trajectory.LFoot_dot(2) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,_ho,0.0,Hz);

                Foot_trajectory.LFoot_euler(1) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,ankle_temp,0.0);
                Foot_trajectory.LFoot_dot(4) = Cubic_dot(_cnt,_T_Start_real+_T_Double1+t_rest_temp,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,0.0,0.0,ankle_temp,0.0,Hz);

            }
            else
            {
                Foot_trajectory.LFoot_euler(1) = Cubic(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-t_rest_temp,_T_Start+_T_Total-_T_rest_last,ankle_temp,0.0,0.0,0.0);
                Foot_trajectory.LFoot_dot(4) = Cubic_dot(_cnt,_T_Start+_T_Total-_T_rest_last-_T_Double2-t_rest_temp,_T_Start+_T_Total-_T_rest_last,ankle_temp,0.0,0.0,0.0,Hz);


                Foot_trajectory.LFoot.translation()(2) = Cubic(_cnt,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_ho,0.0,target_swing_foot(2),0.0);
                Foot_trajectory.LFoot_dot(2) = Cubic_dot(_cnt,_T_Start_real+_T_Double1+(_T_Total-_T_rest_init-_T_rest_last-_T_Double1-_T_Double2-_T_Imp)/2.0,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_ho,0.0,target_swing_foot(2),0.0,Hz);
            }

            for(int i=0; i<2; i++)
            {
                Foot_trajectory.LFoot_euler(0) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(0+3),0.0);
                Foot_trajectory.LFoot_dot(0+3) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(0+3),0.0,Hz);


                Foot_trajectory.LFoot.translation()(i) = Cubic(_cnt,_T_Start_real+_T_Double1+t_rest_temp+0.1*Hz,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.LFoot.translation()(i),0.0,target_swing_foot(i),0.0);
                Foot_trajectory.LFoot_dot(i) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.LFoot.translation()(i),0.0,target_swing_foot(i),0.0,Hz);
            }

          //  for(int i=0; i<3; i++)
          //  {
          //      Foot_trajectory.LFoot_euler(i) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(i+3),0.0);
          //      Foot_trajectory.LFoot_dot(i+3) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,0.0,0.0,target_swing_foot(i+3),0.0,Hz);
          //  }


                 Foot_trajectory.LFoot_euler(2) = Cubic(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.LFoot_euler(2),0.0,target_swing_foot(5),0.0);
                Foot_trajectory.LFoot_dot(5) = Cubic_dot(_cnt,_T_Start_real+_T_Double1,_T_Start+_T_Total-_T_rest_last-_T_Double2-_T_Imp,_init_Foot_trajectory.LFoot_euler(2),0.0,target_swing_foot(5),0.0,Hz);


            Foot_trajectory.LFoot.linear() = Rotate_with_Z(Foot_trajectory.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory.LFoot_euler(1))*Rotate_with_X(Foot_trajectory.LFoot_euler(0));
        }
        else
        {
            Foot_trajectory.LFoot.translation() = _init_Foot_trajectory.LFoot.translation();
            Foot_trajectory.LFoot.linear() = _init_Foot_trajectory.LFoot.linear();
            Foot_trajectory.LFoot_euler = _init_Foot_trajectory.LFoot_euler;
            Foot_trajectory.LFoot_dot.setZero();

            Foot_trajectory.RFoot.translation() = _init_Foot_trajectory.RFoot.translation();
            Foot_trajectory.RFoot.linear() = _init_Foot_trajectory.RFoot.linear();
            Foot_trajectory.RFoot_euler = _init_Foot_trajectory.RFoot_euler;
            Foot_trajectory.RFoot_dot.setZero();
        }
    }
    else
    {
        if(_foot_step(_step_number,6) == 1) // �޹� ����
        {
            Foot_trajectory.LFoot.translation() = _init_Foot_trajectory.LFoot.translation();
            Foot_trajectory.LFoot.translation()(2) = 0.0;
            Foot_trajectory.LFoot_euler = _init_Foot_trajectory.LFoot_euler;
            Foot_trajectory.LFoot_euler(0) = 0.0;
            Foot_trajectory.LFoot_euler(1) = 0.0;
            Foot_trajectory.LFoot.linear() = Rotate_with_Z(Foot_trajectory.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory.LFoot_euler(1))*Rotate_with_X(Foot_trajectory.LFoot_euler(0));
            Foot_trajectory.LFoot_dot.setZero();

            for(int i=0; i<3; i++)
            {
                Foot_trajectory.RFoot.translation()(i) = target_swing_foot(i);
                Foot_trajectory.RFoot_euler(i) = target_swing_foot(i+3);
            }
            Foot_trajectory.RFoot_dot.setZero();

            Foot_trajectory.RFoot.linear() = Rotate_with_Z(Foot_trajectory.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory.RFoot_euler(1))*Rotate_with_X(Foot_trajectory.RFoot_euler(0));
        }
        else if (_foot_step(_step_number,6) == 0)
        {
            Foot_trajectory.RFoot.translation() = _init_Foot_trajectory.RFoot.translation();
            Foot_trajectory.RFoot.translation()(2) = 0.0;
            //Foot_trajectory.RFoot.linear() = _init_Foot_trajectory.RFoot.linear();
            Foot_trajectory.RFoot_euler = _init_Foot_trajectory.RFoot_euler;
	    Foot_trajectory.RFoot_euler(0) = 0.0;
	    Foot_trajectory.RFoot_euler(1) = 0.0;
            Foot_trajectory.RFoot_dot.setZero();
 Foot_trajectory.RFoot.linear() = Rotate_with_Z(Foot_trajectory.RFoot_euler(2))*Rotate_with_Y(Foot_trajectory.RFoot_euler(1))*Rotate_with_X(Foot_trajectory.RFoot_euler(0));


            for(int i=0; i<3; i++)
            {
                Foot_trajectory.LFoot.translation()(i) = target_swing_foot(i);
                Foot_trajectory.LFoot_euler(i) = target_swing_foot(i+3);
            }
            Foot_trajectory.LFoot_dot.setZero();
            Foot_trajectory.LFoot.linear() = Rotate_with_Z(Foot_trajectory.LFoot_euler(2))*Rotate_with_Y(Foot_trajectory.LFoot_euler(1))*Rotate_with_X(Foot_trajectory.LFoot_euler(0));
        }
        else
        {
            Foot_trajectory.LFoot.translation() = _init_Foot_trajectory.LFoot.translation();
            Foot_trajectory.LFoot.linear() = _init_Foot_trajectory.LFoot.linear();
            Foot_trajectory.LFoot_euler = _init_Foot_trajectory.LFoot_euler;
            Foot_trajectory.LFoot_dot.setZero();

            Foot_trajectory.RFoot.translation() = _init_Foot_trajectory.RFoot.translation();
            Foot_trajectory.RFoot.linear() = _init_Foot_trajectory.RFoot.linear();
            Foot_trajectory.RFoot_euler = _init_Foot_trajectory.RFoot_euler;
            Foot_trajectory.RFoot_dot.setZero();
        }
    }
}

void Pattern_generator::Ref_ZMP_update()
{	cout << "ZMP_Update" << endl;
    ////////////////////////ZMP ������ frame �� ���� //////////////////////////////////
    int planning_step_number = 3; // ���ܼ�

    int Nom_size = 0;

    if(_step_number >= _step_total_number-planning_step_number)
        Nom_size = (_T_Last-_T_Start+1)*(_step_total_number-_step_number)+20*Hz;
    else
        Nom_size = (_T_Last-_T_Start+1)*(planning_step_number);

    if(_step_number == 0)
        Nom_size = Nom_size + _T_temp+1;

    ////////////////////�ʱ� �� �ġ �� ��������̼� ����///////////////////////
    if(_foot_step(0,6) == 1)
    {
        for(int i=0; i<2; i++)
            _initial_global_support_foot(i) = _init_info._XL_global_init.translation()(i);
        for(int i=0; i<3; i++)
            _initial_global_support_foot(i+3) = _init_info._XL_global_euler_init(i);

        for(int i=0; i<2; i++)
            _initial_global_swing_foot(i) = _init_info._XR_global_init.translation()(i);
        for(int i=0; i<3; i++)
            _initial_global_swing_foot(i+3) = _init_info._XR_global_euler_init(i);

	_initial_global_support_foot(0) = 0.0;
	_initial_global_swing_foot(0) = 0.0;
    }
    else
    {
        for(int i=0; i<2; i++)
            _initial_global_support_foot(i) = _init_info._XR_global_init.translation()(i);
        for(int i=0; i<3; i++)
            _initial_global_support_foot(i+3) = _init_info._XR_global_euler_init(i);

        for(int i=0; i<2; i++)
            _initial_global_swing_foot(i) = _init_info._XL_global_init.translation()(i);
        for(int i=0; i<3; i++)
            _initial_global_swing_foot(i+3) = _init_info._XL_global_euler_init(i);

    _initial_global_support_foot(0) = 0.0;
	_initial_global_swing_foot(0) = 0.0;
    }


    ///////////////////////ZMP ���� ����////////////////////////////////
    _Ref_ZMP.resize(Nom_size,2);
    _Ref_ZMP.setZero();

    _foot_step_localframe.resize(_step_total_number,7);
    _foot_step_localframe.setZero();
    _foot_step_localframe_offset.resize(_step_total_number,7);
    _foot_step_localframe_offset.setZero();

    GlobaltoLocal_footstep();

    ZMP_offset_planning(_foot_step_localframe,_foot_step_localframe_offset);

    ZMP_Generator(Nom_size,_foot_step_localframe_offset,_Ref_ZMP,planning_step_number);
}



void Pattern_generator::Ref_COM_update_local()
{
   // _COM_offset(0) = -0.08;
    // �κ� ���� �� cnt=0 (ùƽ)���� ���� �Ǵ� �κ��� COM or Trunk�ġ
    if(_com_control_flag == true)
    {
        _xi(0) = _init_info._COM_support_init(0);
        _yi(0) = _init_info._COM_support_init(1);
    }
    else
    {
        _xi(0) = _init_info._trunk_support_init.translation()(0)+_COM_offset(0);
        _yi(0) = _init_info._trunk_support_init.translation()(1)+_COM_offset(1);
    }

    //�κ��� step��ȯ�� com�� �ӵ� ���� ������ ������(������ �����)
    if (_cnt == _T_Start && _step_number != 0)
    {
        Vector3D COB_vel_prev;
        Vector3D COB_vel;
        Vector3D COB_acc_prev;
        Vector3D COB_acc;

        if(_step_number == 1) //���� ������� init_support �̹Ƿ�
        {
            Matrix3D temp;
            temp = Rotate_with_Z(-_initial_global_support_foot(5));

            COB_vel_prev(0) = _xs(1);
            COB_vel_prev(1) = _ys(1);
            COB_vel_prev(2) = 0.0;
            COB_vel = temp*COB_vel_prev;

            COB_acc_prev(0) = _xs(2);
            COB_acc_prev(1) = _ys(2);
            COB_acc_prev(2) = 0.0;
            COB_acc = temp*COB_acc_prev;
        }
        else
        {
            Matrix3D temp;
            temp = Rotate_with_Z(-_foot_step_localframe(_step_number-1,5)); ////////////////�ٽ� �����غ��� ��ǥ��

            COB_vel_prev(0) = _xs(1);
            COB_vel_prev(1) = _ys(1);
            COB_vel_prev(2) = 0.0;
            COB_vel = temp*COB_vel_prev;

            COB_acc_prev(0) = _xs(2);
            COB_acc_prev(1) = _ys(2);
            COB_acc_prev(2) = 0.0;
            COB_acc = temp*COB_acc_prev;
        }

        _xs(1) = COB_vel(0);
        _ys(1) = COB_vel(1);
        _xs(2) = COB_acc(0);
        _ys(2) = COB_acc(1);
    }

    //COM �ġ ���
    if(_COM_update_flag == true)
    {
        if(_com_control_flag == true)
        {
            _xs(0) = _init_COM._COM(0);//+_xs(1)*1.0/Hz;
            _ys(0) = _init_COM._COM(1);//+_ys(1)*1.0/Hz;
        }
        else
        {
            _xs(0) = _init_COM._Trunk.translation()(0)+ _COM_offset(0);// + _xs(1)*1.0/Hz;
            _ys(0) = _init_COM._Trunk.translation()(1)+ _COM_offset(1);// + _ys(1)*1.0/Hz;
        }
    }

    double x_err;
    double y_err;

    ////PreviewControl_basic(_cnt-_init_info.t, _K, Frequency2,  16*Hz2/10, 0.6802, _Gx, _Gi, _Gp_I, _A, _BBB, &_sum_px_err, &_sum_py_err, _px_ref, _py_ref, _xi, _yi, _xs, _ys, _xd, _yd, x_err, y_err);
    modified_PreviewControl(_T_Double1, _T_Total);
    _xs=_xd;
    _ys=_yd;

    //Reference ZMP �� ���� �ð�� ������� �������� ZMP�� �����ϱ� ������ start_time��� ���ߵ�.
    if(_step_number == 0)
        start_time = 0;
    else
        start_time = _T_Start;

    _ZMP_desired(0) = _Ref_ZMP(_cnt-start_time,0);
    _ZMP_desired(1) = _Ref_ZMP(_cnt-start_time,1);

    if(_com_control_flag == true)
    {
        _COM_desired(0) = _xd(0);
        _COM_desired(1) = _yd(0);
        //_COM_desired(2) = _init_info._COM_support_init(2);
        //_COM_desired(2) = Cubic(_cnt,_T_Start,_T_Start+_T_Double1,_init_COM._COM(2),0.0,_init_info._COM_support_init(2),0.0);
        _COM_desired(2) = Cubic(_cnt,_T_Start,_T_Start_real,_init_COM._Trunk.translation()(2),0.0,_init_info._trunk_support_init.translation()(2),0.0);
        double k= 100.0;
        p_ref(0) = _xd(1)+k*(_xd(0)-_COM_real_support(0));
        p_ref(1) = _yd(1)+k*(_yd(0)-_COM_real_support(1));
        p_ref(2) = k*(_COM_desired(2)-_COM_real_support(2));
        L_ref.setZero();
    }
    else
    {
        _COM_desired(0) = _xd(0);
        _COM_desired(1) = _yd(0);
        _COM_desired(2) = Cubic(_cnt,_T_Start,_T_Start_real,_init_COM._Trunk.translation()(2),0.0,_init_info._trunk_support_init.translation()(2),0.0);

        double k= 100.0;
        p_ref(0) = _xd(1)+k*(_xd(0)-_COM_real_support(0));
        p_ref(1) = _yd(1)+k*(_yd(0)-_COM_real_support(1));
        p_ref(2) = k*(_COM_desired(2)-_COM_real_support(2));
        L_ref.setZero();
    }
}



void Pattern_generator::modified_PreviewControl(int td, int TT)
{
    if(_cnt == 0)
        equation(1.0/Hz, 16*Hz/10, _init_info._COM_support_init(2), _K, _Gi, _Gp_I, _Gx, _A, _BBB);

    if(_step_number == 0)
        start_time = 0;
    else
        start_time = _T_Start;

    int size = _Ref_ZMP.col(1).size();

    _px_ref.resize(size);
    _py_ref.resize(size);
    for (int i=0; i<size; i++)
    {
        _px_ref(i) = _Ref_ZMP(i,0);
        _py_ref(i) = _Ref_ZMP(i,1);
    }

    _ux_1 = 0.0;
    _uy_1 = 0.0;
    PreviewControl(_cnt-start_time, _K, 1/Hz,  16*Hz/10, _init_info._COM_support_init(2), _Gx, _Gi, _Gp_I, _A, _BBB, _px_ref, _py_ref, _xi, _yi, _xs, _ys, _xd, _yd, _ux_1, _uy_1, _ux, _uy);

    MatrixXD C(1,3);
    C.setZero();
    C(0,0) = 1.0;
    C(0,2) = -_init_info._COM_support_init(2)/Gravity;

    MatrixXD xs_matrix(3,1);
    for (int i=0; i<3; i++)
        xs_matrix(i,0) = _xd(i);
    MatrixXD ys_matrix(3,1);
    for (int i=0; i<3; i++)
        ys_matrix(i,0) = _yd(i);

    MatrixXD est_zmp_error_x(1,1);
    est_zmp_error_x = C*xs_matrix;
    MatrixXD est_zmp_error_y(1,1);
    est_zmp_error_y = C*ys_matrix;

    double rx, ry;
  //  VectorXD permision_px_ref(16*Hz/10+1);
  //  VectorXD permision_py_ref(16*Hz/10+1);
    rx = 0.0;
    ry = 0.0;

    //if (_px_ref(_cnt-_init_info.t) < est_zmp_error_x(0,0) && _py_ref(_cnt-_init_info.t) < est_zmp_error_y(0,0))
    //{
    //	int sign_x = 0;
    //	int sign_y = 0;
    //	calculating_R(T_Double, T_Total, xs_matrix, ys_matrix, rx, ry, permision_px_ref, permision_py_ref,sign_x, sign_y);
    //}
    //else if (_px_ref(_cnt-_init_info.t) < est_zmp_error_x(0,0) && _py_ref(_cnt-_init_info.t) >= est_zmp_error_y(0,0))
    //{
    //	int sign_x = 0;
    //	int sign_y = 1;
    //	calculating_R(T_Double, T_Total, xs_matrix, ys_matrix, rx, ry, permision_px_ref, permision_py_ref,sign_x, sign_y);
    //}
    //else if (_px_ref(_cnt-_init_info.t) >= est_zmp_error_x(0,0) && _py_ref(_cnt-_init_info.t) < est_zmp_error_y(0,0))
    //{
    //	int sign_x = 1;
    //	int sign_y = 0;
    //	calculating_R(T_Double, T_Total, xs_matrix, ys_matrix, rx, ry, permision_px_ref, permision_py_ref,sign_x, sign_y);
    //}
    //else if (_px_ref(_cnt-_init_info.t) >= est_zmp_error_x(0,0) && _py_ref(_cnt-_init_info.t) >= est_zmp_error_y(0,0))
    //{
    //	int sign_x = 1;
    //	int sign_y = 1;
    //	calculating_R(T_Double, T_Total, xs_matrix, ys_matrix, rx, ry, permision_px_ref, permision_py_ref,sign_x, sign_y);
    //}
    //else
    //	printf("1");

    //for (int i=0; i<=16*Hz/10; i++)
    //{
    //	modified_px_ref(_cnt-_init_info.t+i) += rx*permision_px_ref(i);
    //	modified_py_ref(_cnt-_init_info.t+i) += ry*permision_py_ref(i);
    //}

    PreviewControl(_cnt-start_time, _K, 1.0/Hz,   16*Hz/10, _init_info._COM_support_init(2), _Gx, _Gi, _Gp_I, _A, _BBB, _px_ref, _py_ref, _xi, _yi, _xs, _ys, _xd, _yd, _ux_1, _uy_1, _ux, _uy);
    _ux_1 = _ux;
    _uy_1 = _uy;

    _xs = _xd;
    _ys = _yd;

    MatrixXD est_zmp(1,1);
    est_zmp(0,0) = C(0,0)*_xs(0)+C(0,1)*_xs(1)+C(0,2)*_xs(2);
}


void Pattern_generator::equation(double dt, int NL, double _COM_init, MatrixXD& K, MatrixXD& Gi, VectorXD& Gp_l, MatrixXD& Gx, MatrixXD& A, MatrixXD& B)
{

    double Zc = _COM_init;


    A(0,0) = 1.0;
    A(1,1) = 1.0;
    A(2,2) = 1.0;
    A(0,1) = dt;
    A(1,2) = dt;
    A(0,2) = dt*dt/2.0;


    B(0,0) = dt*dt*dt/6.0;
    B(1,0) = dt*dt/2.0;
    B(2,0) = dt;

    MatrixXD C(1,3);
    C.setZero();
    C(0,0) = 1.0;
    C(0,2) = -Zc/Gravity;

    MatrixXD B_(4,1);
    B_.setZero();
    MatrixXD CB;
    CB.resize(1,1);
    CB(0,0) = 0.0;
    CB = C*B;
    B_(0,0) = CB(0,0);
    B_(1,0) = B(0,0);
    B_(2,0) = B(1,0);
    B_(3,0) = B(2,0);

    MatrixXD B_T(1,4);
    B_T = B_.transpose();

    MatrixXD F_(4,3);
    F_.setZero();
    MatrixXD CA = C*A;
    F_(0,0) = CA(0,0);
    F_(0,1) = CA(0,1);
    F_(0,2) = CA(0,2);
    F_(1,0) = A(0,0);
    F_(1,1) = A(0,1);
    F_(1,2) = A(0,2);
    F_(2,0) = A(1,0);
    F_(2,1) = A(1,1);
    F_(2,2) = A(1,2);
    F_(3,0) = A(2,0);
    F_(3,1) = A(2,1);
    F_(3,2) = A(2,2);

    MatrixXD I_(4,1);
    I_.setZero();
    I_(0,0) = 1.0;

    MatrixXD A_(4,4);
    A_.setZero();
    A_(0,0) = I_(0,0);
    A_(1,0) = I_(1,0);
    A_(2,0) = I_(2,0);
    A_(3,0) = I_(3,0);
    A_(0,1) = F_(0,0);
    A_(0,2) = F_(0,1);
    A_(0,3) = F_(0,2);
    A_(1,1) = F_(1,0);
    A_(1,2) = F_(1,1);
    A_(1,3) = F_(1,2);
    A_(2,1) = F_(2,0);
    A_(2,2) = F_(2,1);
    A_(2,3) = F_(2,2);
    A_(3,1) = F_(3,0);
    A_(3,2) = F_(3,1);
    A_(3,3) = F_(3,2);

    MatrixXD Qe(1,1);
    Qe(0,0) = 1.0;

    MatrixXD R(1,1);
    R(0,0) = 0.000001;

    MatrixXD Qx(3,3);
    Qx.setZero();

    MatrixXD Q_(4,4);
    Q_.setZero();
    Q_(0,0) = Qe(0,0);
    Q_(1,1) = Qx(0,0);
    Q_(1,2) = Qx(0,1);
    Q_(1,3) = Qx(0,2);
    Q_(2,1) = Qx(1,0);
    Q_(2,2) = Qx(1,1);
    Q_(2,3) = Qx(1,2);
    Q_(3,1) = Qx(2,0);
    Q_(3,2) = Qx(2,1);
    Q_(3,3) = Qx(2,2);

    K = dare(A_,B_,R,Q_);

    MatrixXD Ac_(4,4);
    Ac_.setZero();
    MatrixXD temp_Mat(1,1);
    temp_Mat.setZero();
    temp_Mat = R+B_T*K*B_;


    MatrixXD temp_Mat_inv(1,1);
    temp_Mat_inv = temp_Mat.inverse();

    Ac_ = A_-B_*temp_Mat_inv*B_T*K*A_;
    MatrixXD Ac_T(4,4);
    Ac_T = Ac_.transpose();


    Gi = temp_Mat_inv*B_T*K*I_;

    Gx = temp_Mat_inv*B_T*K*F_;


    MatrixXD X_l(4,NL);
    X_l.setZero();
    MatrixXD X_l_column(4,1);
    X_l_column.setZero();
    X_l_column = -Ac_T*K*I_; //init column vector of X(l)
    for(int i=0; i<NL; i++)
    {
        X_l.block<4,1>(0,i) = X_l_column;
        X_l_column = Ac_T*X_l_column;
    }

    VectorXD Gp_l_column(1);
    Gp_l_column(0) = -Gi(0,0);
    for(int i=0; i<NL; i++)
    {
        Gp_l.segment(i,1) = Gp_l_column;
//		Gp_l.set(i,1,Gp_l_column);
        Gp_l_column = temp_Mat_inv*B_T*X_l.col(i);
    }
}



MatrixXD Pattern_generator::dare(MatrixXD& A, MatrixXD& B, MatrixXD& R, MatrixXD& Q)
{
    //A,Q,X:n*n, B:n*m, R:m*m
    //A.print(_T("A"));
    //B.print(_T("B"));
    //R.print(_T("R"));
    //Q.print(_T("Q"));

    int n=A.rows(); //number of rows
    int	m=B.cols(); //number of columns
    MatrixXD Z11(n,n);
    MatrixXD Z12(n,n);
    MatrixXD Z21(n,n);
    MatrixXD Z22(n,n);
    MatrixXD temp1(m,m);
    MatrixXD temp2(n,n);
    MatrixXD temp3(m,n);

    Z11 = A.inverse();
    temp1 = R.inverse();
    Z21=Q*Z11;

    MatrixXD B_T;
    B_T = B.transpose();
    temp2=B*temp1*B_T;     //B*inv(R)*B'
    Z12=Z11*temp2;
    MatrixXD A_T;
    A_T = A.transpose();
    Z22 = A_T+Z21*temp2;
    //construct the Z with Z11,Z12,Z21,Z22

    MatrixXD Z(2*n,2*n);
    Z.setZero();

    // ggory15
    Z.topLeftCorner(n,n) = Z11;
    Z.topRightCorner(n,n) = Z12;
    Z.bottomLeftCorner(n,n) = Z21;
    Z.bottomRightCorner(n,n) = Z22;


    /*
    Z.set(0,0,n,n,Z11);
    Z.set(0,n,n,n,Z12);
    Z.set(n,0,n,n,Z21);
    Z.set(n,n,n,n,Z22);
    */

    //Z.print(_T("Z"));
    using namespace std;

    std::vector<double> eigVal_real(2*n); //eigen value�� real��
    std::vector<double> eigVal_img(2*n); //eigen value�� img��
    std::vector<VectorXD> eigVec_real(2*n); //eigen vector�� real��
    std::vector<VectorXD> eigVec_img(2*n); //eigen vector�� img��

    for(int i=0;i<2*n;i++) //���� �ʱ�ȭ
    {
        eigVec_real[i].resize(2*n);
        eigVec_real[i].setZero();
        eigVec_img[i].resize(2*n);
        eigVec_img[i].setZero();
    }

    VectorXD deigVal_real(2*n);
    VectorXD deigVal_img(2*n);
    deigVal_real.setZero();
    deigVal_img.setZero();
    MatrixXD deigVec_real(2*n,2*n);
    MatrixXD deigVec_img(2*n,2*n);
    deigVec_real.setZero();
    deigVec_img.setZero();

    MatrixXD Z_evr = Z; // �ӽ� ����, rmath�� evr� �ϸ� ���� Z matrix�� ������

    deigVal_real = Z_evr.eigenvalues().real();
    deigVal_img = Z_evr.eigenvalues().imag();

    Eigen::EigenSolver<MatrixXD> es(Z_evr);

    //Matrix3D ones = Matrix3D::Ones(3,3);
    //EigenSolver<Matrix3D> es(ones);
    //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << es.eigenvectors().col(1) << endl;
    //	Z_evr.evr(eigVal_real, eigVal_img, eigVec_real,eigVec_img); //right eigen vector�� ����



    for(int i=0;i<2*n; i++)
    {
        for(int ii=0; ii<2*n; ii++)
        {
            deigVec_real(ii,i) = es.eigenvectors().col(i)(ii).real();
            deigVec_img(ii,i) = es.eigenvectors().col(i)(ii).imag();
        }
    }

    //deigVal_real.print(_T("eigen value real"));
    //deigVal_img.print(_T("eigen value img"));
    //deigVec_real.print(_T("eigen vector real"));
    //deigVec_img.print(_T("eigen vector img"));

    //Order the eigenvectors
    //move e-vectors correspnding to e-value outside the unite circle to the left
    MatrixXD tempZ_real(2*n,n);
    MatrixXD tempZ_img(2*n,n);
    tempZ_real.setZero();
    tempZ_img.setZero();

    int c1=0;

    for (int i=0;i<2*n;i++)
    {
        if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
        {
            for(int j=0; j<2*n; j++)
            {
                tempZ_real(j,c1) = deigVec_real(j,i);
                tempZ_img(j,c1) = deigVec_img(j,i);
            }
            c1++;
        }
    }

    using namespace Eigen;

    MatrixXcd tempZ_comp(2*n,n);
    for(int i=0;i<2*n;i++)
    {
        for(int j=0;j<n;j++)
        {
            tempZ_comp.real()(i,j) = tempZ_real(i,j);
            tempZ_comp.imag()(i,j) = tempZ_img(i,j);
        }
    }

    MatrixXcd U11(n,n);
    MatrixXcd U21(n,n);
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            U11(i,j) = tempZ_comp(i,j);
            U21(i,j) = tempZ_comp(i+n,j);
        }
    }

    MatrixXcd U11_inv(n,n);
    U11_inv = U11.inverse();

    MatrixXcd X(n,n);
    X = U21*(U11_inv);

    MatrixXD X_sol(n,n);

    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            X_sol(i,j) = X.real()(i,j);
        }
    }

    //	X_sol.print(_T(""));

    return X_sol;
}

void Pattern_generator::PreviewControl(int k, MatrixXD& K, double dt, int NL, double _COM_init, MatrixXD& Gx, MatrixXD& Gi, VectorXD& Gp_l, MatrixXD& A, MatrixXD& B, VectorXD& px_ref, VectorXD& py_ref, Vector3D xi, Vector3D yi, Vector3D xs, Vector3D ys, Vector3D &xd, Vector3D &yd, double ux_1 , double uy_1 , double &ux, double &uy)
{

    double Zc = _COM_init;


    Vector3D X;
    X.setZero();
    Vector3D Y;
    Y.setZero();
    if(k== 0 && _step_number == 0)
    {
        X = xi;
        X(1) = 0.0;
        X(2) = 0.0;
        Y = yi;
        Y(1) = 0.0;
        Y(2) = 0.0;
    }
    else
    {
        X = xi;
        Y = yi;

        X = _xs;
        Y = _ys;
    }

    Vector3D X_1;
    Vector3D Y_1;
    X_1.setZero();
    Y_1.setZero();

    X_1(0) = X(0)-X(1)*dt;
    X_1(1) = X(1)-X(2)*dt;
    X_1(2) = X(2);
    Y_1(0) = Y(0)-Y(1)*dt;
    Y_1(1) = Y(1)-Y(2)*dt;
    Y_1(2) = Y(2);

    MatrixXD K_(4,4);
    K_=K;

    MatrixXD C(1,3);
    C.setZero();
    C(0,0) = 1.0;
    C(0,2) = -Zc/Gravity;

    double xzmp_err =0.0, yzmp_err = 0.0;

    VectorXD px(1);
    px = C*X;
    VectorXD py(1);
    py = C*Y;

    xzmp_err = px(0) - px_ref(k);
    yzmp_err = py(0) - py_ref(k);

    double sum_Gp_px_ref = 0.0, sum_Gp_py_ref = 0.0;
    for(int i = 0; i < NL; i++)
    {
        sum_Gp_px_ref = sum_Gp_px_ref + Gp_l(i)*(px_ref(k+1+i)-px_ref(k+i));
        sum_Gp_py_ref = sum_Gp_py_ref + Gp_l(i)*(py_ref(k+1+i)-py_ref(k+i));
    }


    VectorXD GxX(1);
    GxX = Gx*(X-X_1);
    VectorXD GxY(1);
    GxY = Gx*(Y-Y_1);

    MatrixXD del_ux(1,1);
    MatrixXD del_uy(1,1);
    del_ux.setZero();
    del_uy.setZero();

    del_ux(0,0) = -(xzmp_err*Gi(0,0))-GxX(0)-sum_Gp_px_ref;
    del_uy(0,0) = -(yzmp_err*Gi(0,0))-GxY(0)-sum_Gp_py_ref;


    ux = ux_1 + del_ux(0,0);
    uy = uy_1 + del_uy(0,0);

    VectorXD ux_vec(1);
    ux_vec(0) = ux;
    VectorXD uy_vec(1);
    uy_vec(0) = uy;

    xd = A*X + B*ux_vec;
    yd = A*Y + B*uy_vec;
}


void Pattern_generator::ZMP_Generator(int Nom_Size, MatrixXD& _foot_step_localframe_offset, MatrixXD& _Ref_ZMP, int planning_step_number)
{

    _Ref_ZMP.resize(Nom_Size,2);

    VectorXD temp_px;
    VectorXD temp_py;

    int index = 0;

    if(_step_number == 0)
    {
        for (int i=0; i<= _T_temp; i++) //200 tick���� ��� ���
        {
            if(i <= 0.5*Hz)
            {

                _Ref_ZMP(i,0) = _init_info._COM_support_init(0)+_COM_offset(0);
                _Ref_ZMP(i,1) = _init_info._COM_support_init(1)+_COM_offset(1);
            }
            else if(i < 1.5*Hz)
            {

                double del_x = i-0.5*Hz;
                _Ref_ZMP(i,0) = _init_info._COM_support_init(0)+_COM_offset(0)-del_x*(_init_info._COM_support_init(0)+_COM_offset(0))/(1.0*Hz);
                _Ref_ZMP(i,1) = _init_info._COM_support_init(1)+_COM_offset(1);
            }
            else
            {

                _Ref_ZMP(i,0) = 0.0;
                _Ref_ZMP(i,1) = _init_info._COM_support_init(1)+_COM_offset(1);
            }



            index++;
        }
    }

    if(_step_number >= _step_total_number-planning_step_number)
    {
        for(int i = _step_number; i<_step_total_number ; i++)
        {
            Onestep_ZMP(_T_rest_init,_T_rest_last,_T_Double1,_T_Double2,_T_Total,i,temp_px,temp_py);

            for (int j=0; j<_T_Total; j++)
            {
                _Ref_ZMP(index+j,0) = temp_px(j);
                _Ref_ZMP(index+j,1) = temp_py(j);
            }
            index = index+_T_Total;
        }

        for (int j=0; j<20*Hz; j++)
        {
            _Ref_ZMP(index+j,0) = _Ref_ZMP(index-1,0);
            _Ref_ZMP(index+j,1) = _Ref_ZMP(index-1,1);
        }
        index = index+20*Hz;
    }
    else
    {
        for(int i=_step_number; i < _step_number+planning_step_number; i++)
        {
            Onestep_ZMP(_T_rest_init,_T_rest_last,_T_Double1,_T_Double2,_T_Total,i,temp_px,temp_py);

            for (int j=0; j<_T_Total; j++)
            {
                _Ref_ZMP(index+j,0) = temp_px(j);
                _Ref_ZMP(index+j,1) = temp_py(j);
            }
            index = index+_T_Total;
        }
    }
}
void Pattern_generator::Onestep_stop_ZMP(double _T_Total,double ZMP_x, double ZMP_y, VectorXD &temp_px,VectorXD& temp_py)
{
    // Left ������ ���� ����� ������ ����
    for (int i=0 ; i<_T_Total ; i++)
    {
        temp_px(i) = ZMP_x;
        temp_py(i) = ZMP_y;
    }
}
void Pattern_generator::Onestep_ZMP(double T_rest_init, double T_rest_last, double T_Double1, double T_Double2, double T_Total, int step_number,VectorXD& temp_px, VectorXD& temp_py)
{
    temp_px.resize(T_Total);
    temp_py.resize(T_Total);
    temp_px.setZero();
    temp_py.setZero();

    double Kx = 0.0;
    double Kx2 = 0.0;
    double Ky = 0.0;
    double Ky2 = 0.0;

    if(step_number == 0)
    {
	//_initial_local_support_foot_offset(0)=0.0;

       // Kx = _initial_local_support_foot_offset(0) - _init_info._COM_support_init(0) - _COM_offset(0);
        Kx = _initial_local_support_foot_offset(0);// - _init_info._COM_support_init(0) - _COM_offset(0);
        Kx2 = (_foot_step_localframe(step_number,0)+_initial_local_support_foot(0))/2.0 - _initial_local_support_foot_offset(0);

        Ky = _initial_local_support_foot_offset(1) - _init_info._COM_support_init(1);
        Ky2 = (_foot_step_localframe(step_number,1)+_initial_local_support_foot(1))/2.0 - _initial_local_support_foot_offset(1);

        for(int i=0; i<T_Total; i++)
        {
            if(i < T_rest_init)
            {
                temp_px(i) = 0.0;//_init_info._COM_support_init(0)+_COM_offset(0);
                temp_py(i) = _init_info._COM_support_init(1)+_COM_offset(1);
            }
            else if(i >= T_rest_init && i < T_rest_init+T_Double1)
            {
              //  temp_px(i) = _init_info._COM_support_init(0)+_COM_offset(0) + Kx/T_Double1*(i+1-T_rest_init);
                temp_py(i) = _init_info._COM_support_init(1)+_COM_offset(1) + Ky/T_Double1*(i+1-T_rest_init);
                temp_px(i) = Kx/T_Double1*(i+1-T_rest_init);
                //temp_py(i) = Ky/T_Double1*(i+1-T_rest_init);
            }
            else if(i>= T_rest_init+T_Double1 & i< T_Total-T_rest_last-T_Double2)
            {
                temp_px(i) = _initial_local_support_foot_offset(0);
                temp_py(i) = _initial_local_support_foot_offset(1);
            }
            else if(i >= T_Total-T_rest_last-T_Double2 && i< T_Total-T_rest_last)
            {
                temp_px(i) = _initial_local_support_foot_offset(0) + Kx2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
                temp_py(i) = _initial_local_support_foot_offset(1) + Ky2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
            }
            else
            {
                temp_px(i) = temp_px(i-1);
                temp_py(i) = temp_py(i-1);
            }
        }
    }
    else if(step_number == 1)
    {
//_initial_local_support_foot(0) = 0.0;

        Kx = _foot_step_localframe_offset(step_number-1,0) - (_foot_step_localframe(step_number-1,0) + _initial_local_support_foot(0))/2.0;
        Kx2 = (_foot_step_localframe(step_number,0)+_foot_step_localframe(step_number-1,0))/2.0 - _foot_step_localframe_offset(step_number-1,0);

        Ky =  _foot_step_localframe_offset(step_number-1,1) - (_foot_step_localframe(step_number-1,1) + _initial_local_support_foot(1))/2.0;
        Ky2 = (_foot_step_localframe(step_number,1)+_foot_step_localframe(step_number-1,1))/2.0 - _foot_step_localframe_offset(step_number-1,1);

        for(int i=0; i<T_Total; i++)
        {
            if(i < T_rest_init)
            {
                temp_px(i) = (_foot_step_localframe(step_number-1,0)+_initial_local_support_foot(0))/2.0;
                temp_py(i) = (_foot_step_localframe(step_number-1,1)+_initial_local_support_foot(1))/2.0;
            }
            else if(i >= T_rest_init && i < T_rest_init+T_Double1)
            {
                temp_px(i) = (_foot_step_localframe(step_number-1,0)+_initial_local_support_foot(0))/2.0 + Kx/T_Double1*(i+1-T_rest_init);
                temp_py(i) = (_foot_step_localframe(step_number-1,1)+_initial_local_support_foot(1))/2.0 + Ky/T_Double1*(i+1-T_rest_init);
            }
            else if(i>= T_rest_init+T_Double1 & i< T_Total-T_rest_last-T_Double2)
            {
                temp_px(i) = _foot_step_localframe_offset(step_number-1,0);
                temp_py(i) = _foot_step_localframe_offset(step_number-1,1);
            }
            else if(i >= T_Total-T_rest_last-T_Double2 && i< T_Total-T_rest_last)
            {
                temp_px(i) = _foot_step_localframe_offset(step_number-1,0) + Kx2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
                temp_py(i) = _foot_step_localframe_offset(step_number-1,1) + Ky2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
            }
            else
            {
                temp_px(i) = temp_px(i-1);
                temp_py(i) = temp_py(i-1);
            }
        }
    }
    else
    {
        Kx = _foot_step_localframe_offset(step_number-1,0) - (_foot_step_localframe(step_number-1,0) + _foot_step_localframe(step_number-2,0))/2.0;
        Kx2 = (_foot_step_localframe(step_number,0)+_foot_step_localframe(step_number-1,0))/2.0 - _foot_step_localframe_offset(step_number-1,0);

        Ky =  _foot_step_localframe_offset(step_number-1,1) - (_foot_step_localframe(step_number-1,1) + _foot_step_localframe(step_number-2,1))/2.0;
        Ky2 = (_foot_step_localframe(step_number,1)+_foot_step_localframe(step_number-1,1))/2.0 -  _foot_step_localframe_offset(step_number-1,1);

        for(int i=0; i<T_Total; i++)
        {
            if(i < T_rest_init)
            {
                temp_px(i) = (_foot_step_localframe(step_number-1,0)+_foot_step_localframe(step_number-2,0))/2.0;
                temp_py(i) = (_foot_step_localframe(step_number-1,1)+_foot_step_localframe(step_number-2,1))/2.0;
            }
            else if(i >= T_rest_init && i < T_rest_init+T_Double1)
            {
                temp_px(i) = (_foot_step_localframe(step_number-1,0)+_foot_step_localframe(step_number-2,0))/2.0 + Kx/T_Double1*(i+1-T_rest_init);
                temp_py(i) = (_foot_step_localframe(step_number-1,1)+_foot_step_localframe(step_number-2,1))/2.0 + Ky/T_Double1*(i+1-T_rest_init);
            }
            else if(i>= T_rest_init+T_Double1 & i< T_Total-T_rest_last-T_Double2)
            {
                temp_px(i) = _foot_step_localframe_offset(step_number-1,0);
                temp_py(i) = _foot_step_localframe_offset(step_number-1,1);
            }
            else if(i >= T_Total-T_rest_last-T_Double2 && i< T_Total-T_rest_last)
            {
                temp_px(i) = _foot_step_localframe_offset(step_number-1,0) + Kx2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
                temp_py(i) = _foot_step_localframe_offset(step_number-1,1) + Ky2/(T_Double2)*(i+1-(T_Total-T_Double2-T_rest_last));
            }
            else
            {
                temp_px(i) = temp_px(i-1);
                temp_py(i) = temp_py(i-1);
            }
        }
    }


}

void Pattern_generator::ZMP_offset_planning(MatrixXD& _foot_step_localframe, MatrixXD& _foot_step_localframe_offset)
{

    _foot_step_localframe_offset = _foot_step_localframe;

    if(_foot_step(0,6) == 1) // �޹�����
    {
        _initial_local_support_foot_offset(1) = _initial_local_support_foot(1) + _ZMP_left_offset;
        _initial_local_swing_foot_offset(1) = _initial_local_swing_foot(1) + _ZMP_right_offset;
    }
    else
    {
        _initial_local_support_foot_offset(1) = _initial_local_support_foot(1) + _ZMP_right_offset;
        _initial_local_swing_foot_offset(1) = _initial_local_swing_foot(1) + _ZMP_left_offset;
    }

    for(int i=0; i<_step_total_number; i++)
    {
        if(_foot_step(i,6) == 0) //����� ����
            _foot_step_localframe_offset(i,1) += _ZMP_left_offset;
        else
            _foot_step_localframe_offset(i,1) += _ZMP_right_offset;
    }
}

void Pattern_generator::GlobaltoLocal_footstep()
{
    HTransform reference;

    if(_step_number == 0)
    {
        if(_foot_step(_step_number,6) == 1) //�޹�����
        {
            reference.translation() = _init_info._XL_global_init.translation();
            reference.translation()(2) = 0;
            reference.linear() = Rotate_with_Z(_init_info._XL_global_euler_init(2));
	    reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = _init_info._XR_global_init.translation();
            reference.translation()(2) = 0;
            reference.linear() = Rotate_with_Z(_init_info._XR_global_euler_init(2));
	    reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = Rotate_with_Z(_foot_step(_step_number-1,5));
        for(int i=0 ;i<3; i++)
            reference.translation()(i) = _foot_step(_step_number-1,i);
    }

    Vector3D temp_local_position;
    temp_local_position.setZero();

    Vector3D temp_global_position;
    temp_global_position.setZero();

    if(_step_number == 0)
    {
        for(int i=0; i<_step_total_number; i++)
        {
            for(int j=0; j<3; j++)
                temp_global_position(j)  = _foot_step(i,j);


            Globalposition(temp_global_position,reference,temp_local_position);

            for(int j=0; j<3; j++)
                _foot_step_localframe(i,j) = temp_local_position(j);

            _foot_step_localframe(i,3) = _foot_step(i,3);
            _foot_step_localframe(i,4) = _foot_step(i,4);
            _foot_step_localframe(i,5) = _foot_step(i,5) - _initial_global_support_foot(5);

        }
    }
    else
    {
        for(int i=0; i<_step_total_number; i++)
        {
            for(int j=0; j<3; j++)
                temp_global_position(j)  = _foot_step(i,j);

            Globalposition(temp_global_position,reference,temp_local_position);

            for(int j=0; j<3; j++)
                _foot_step_localframe(i,j) = temp_local_position(j);

            _foot_step_localframe(i,3) = _foot_step(i,3);
            _foot_step_localframe(i,4) = _foot_step(i,4);
            _foot_step_localframe(i,5) = _foot_step(i,5) - _foot_step(_step_number-1,5);

        }
    }

    for(int j=0;j<3;j++)
        temp_global_position(j) = _initial_global_swing_foot(j);

    Globalposition(temp_global_position,reference,temp_local_position);
    for(int j=0;j<3;j++)
        _initial_local_swing_foot(j) = temp_local_position(j);

    _initial_local_swing_foot(3) = _initial_global_swing_foot(3);
    _initial_local_swing_foot(4) = _initial_global_swing_foot(4);

    if(_step_number == 0)
        _initial_local_swing_foot(5) = _initial_global_swing_foot(5) - _initial_global_support_foot(5);
    else
        _initial_local_swing_foot(5) = _initial_global_swing_foot(5) - _foot_step(_step_number-1,5);



    for(int j=0;j<3;j++)
        temp_global_position(j) = _initial_global_support_foot(j);

    Globalposition(temp_global_position,reference,temp_local_position);
    for(int j=0;j<3;j++)
        _initial_local_support_foot(j) = temp_local_position(j);

    _initial_local_support_foot(3) = _initial_global_support_foot(3);
    _initial_local_support_foot(4) = _initial_global_support_foot(4);

    if(_step_number == 0)
        _initial_local_support_foot(5) = _initial_global_support_foot(5) - _initial_global_support_foot(5);
    else
        _initial_local_support_foot(5) = _initial_global_support_foot(5) - _foot_step(_step_number-1,5);

}


void Pattern_generator::Pattern_generator_initialize()
{
    _COM_intergral_error.setZero();
    _COM_modification.setZero();
    _initial_global_support_foot.setZero();
    _initial_global_swing_foot.setZero();

    _initial_local_support_foot.setZero();
    _initial_local_swing_foot.setZero();


    _initial_local_support_foot_offset.setZero();
    _initial_local_swing_foot_offset.setZero();


    _xi.setZero();
    _yi.setZero();
    _xs.setZero();
    _ys.setZero();
    _xd.setZero();
    _yd.setZero();

    _COM_offset.setZero();
    _ux_1 = 0.0;
    _uy_1 = 0.0;
    _ux = 0.0;
    _uy = 0.0;

    _K.resize(4,4);
    _K.setZero();

    _Gi.resize(1,1);
    _Gi.setZero();

    _Gp_I.resize(16*Hz/10);
    _Gp_I.setZero();

    _Gx.resize(1,3);
    _Gx.setZero();

    _A.resize(3,3);
    _A.setZero();

    _BBB.resize(3,1);
    _BBB.setZero();
}
