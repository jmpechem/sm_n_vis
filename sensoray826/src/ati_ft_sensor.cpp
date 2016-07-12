#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>

#include "sensoray826.h"


const double SAMPLE_RATE = 1000; // Hz

enum SLOT_TIME {NONE = 0, DEFAULT = 50};

class atiFTSensorROS : public sensoray826_dev
{

    // SLOTs
    static const SLOTATTR slotAttrs[16];

    enum AD_INDEX {LEFT_FOOT = 0, RIGHT_FOOT = 8};

    // FT Datas

    const static double calibrationMatrixLFoot[6][6];
    const static double calibrationMatrixRFoot[6][6];

    double leftFootAxisData[6];
    double rightFootAxisData[6];
    double leftFootAxisData_prev[6];
    double rightFootAxisData_prev[6];

    double leftFootBias[6];
    double rightFootBias[6];

    bool isCalibration;
    double dCalibrationTime;

    double _calibLFTData[6];
    double _calibRFTData[6];
    int _calibTimeIndex;
    int _calibMaxIndex;


    // ROS
    ros::Rate rate;
    ros::Publisher leftFootFTPublisher;
    ros::Publisher rightFootFTPublisher;
    ros::Subscriber calibSubscriber;
    geometry_msgs::WrenchStamped leftFootMsg;
    geometry_msgs::WrenchStamped rightFootMsg;

    int stampCount;


    void initCalibration()
    {
        for(int i=0; i<6; i++)
        {
            _calibLFTData[i] = 0.0;
            _calibRFTData[i] = 0.0;
        }
        _calibTimeIndex = 0;
        _calibMaxIndex = dCalibrationTime * SAMPLE_RATE;
        ROS_INFO("FT sensor calibration start... time = %.1lf sec, total %d samples ", dCalibrationTime, _calibMaxIndex);
    }

    void computeFTData()
    {
        if(isCalibration)
        {
            if(_calibTimeIndex < _calibMaxIndex)
            {
                for(int i=0; i<6; i++)
                {
                    double _lf = 0.0;
                    double _rf = 0.0;
                    for(int j=0; j<6; j++)
                    {
                        _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                        _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                    }
                    _calibLFTData[i] += _lf / _calibMaxIndex;
                    _calibRFTData[i] += _rf / _calibMaxIndex;
                }
                _calibTimeIndex++;
            }
            else
            {
                isCalibration = false;
                for(int i=0; i<6; i++)
                {
                    leftFootBias[i] = _calibLFTData[i];
                    rightFootBias[i] = _calibRFTData[i];
                }

                ROS_INFO("LFT bias data = %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf",
                         leftFootBias[0], leftFootBias[1], leftFootBias[2], leftFootBias[3], leftFootBias[4], leftFootBias[5]);
                ROS_INFO("RFT bias data = %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf",
                         rightFootBias[0], rightFootBias[1], rightFootBias[2], rightFootBias[3], rightFootBias[4], rightFootBias[5]);

            }
        }
        else
        {
            for(int i=0; i<6; i++)
            {
                double _lf = 0.0;
                double _rf = 0.0;
                for(int j=0; j<6; j++)
                {
                    _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                    _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                }

                _lf -= leftFootBias[i];
                _rf -= rightFootBias[i];

                leftFootAxisData[i] = lowPassFilter(_lf, leftFootAxisData_prev[i], 1.0 / SAMPLE_RATE, 0.05);
                rightFootAxisData[i] = lowPassFilter(_rf, rightFootAxisData_prev[i], 1.0/ SAMPLE_RATE,0.05);
                leftFootAxisData_prev[i] = leftFootAxisData[i];
                rightFootAxisData_prev[i] = rightFootAxisData[i];
            }
        }
    }
    void publishFTData()
    {
        leftFootMsg.wrench.force.x = leftFootAxisData[0];
        leftFootMsg.wrench.force.y = leftFootAxisData[1];
        leftFootMsg.wrench.force.z = leftFootAxisData[2];
        leftFootMsg.wrench.torque.x = leftFootAxisData[3];
        leftFootMsg.wrench.torque.y = leftFootAxisData[4];
        leftFootMsg.wrench.torque.z = leftFootAxisData[5];

        rightFootMsg.wrench.force.x = rightFootAxisData[0];
        rightFootMsg.wrench.force.y = rightFootAxisData[1];
        rightFootMsg.wrench.force.z = rightFootAxisData[2];
        rightFootMsg.wrench.torque.x = rightFootAxisData[3];
        rightFootMsg.wrench.torque.y = rightFootAxisData[4];
        rightFootMsg.wrench.torque.z = rightFootAxisData[5];

        leftFootMsg.header.stamp = ros::Time::now();
        rightFootMsg.header.stamp = leftFootMsg.header.stamp;
        leftFootFTPublisher.publish(leftFootMsg);
        rightFootFTPublisher.publish(rightFootMsg);
        stampCount++;
    }

    void calibCallback(const std_msgs::Float32ConstPtr msg)
    {
        dCalibrationTime = msg->data;
        initCalibration();
        isCalibration = true;
        //calibration(msg->data);
    }

public:
    atiFTSensorROS(ros::NodeHandle &nh) : sensoray826_dev(), isCalibration(false), stampCount(0), rate(SAMPLE_RATE)
    {
        // ROS publish
        leftFootFTPublisher = nh.advertise<geometry_msgs::WrenchStamped>("ati_ft_sensor/left_foot_ft",5);
        rightFootFTPublisher = nh.advertise<geometry_msgs::WrenchStamped>("ati_ft_sensor/right_foot_ft",5);
        calibSubscriber = nh.subscribe("ati_ft_sensor/calibration", 1, &atiFTSensorROS::calibCallback, this);

        leftFootMsg.header.frame_id = "LeftFootFT";
        rightFootMsg.header.frame_id = "RightFootFT";


        // daq open
        sensoray826_dev::open();
        sensoray826_dev::analogSingleSamplePrepare(slotAttrs, 16);
    }

    void loop()
    {
        analogOversample();
        computeFTData();
        publishFTData();
    }

};


const SLOTATTR atiFTSensorROS::slotAttrs[16] = {
    {0, DEFAULT}, {1, DEFAULT}, {2, DEFAULT}, {3, DEFAULT},
    {4, DEFAULT}, {5, DEFAULT}, {6, DEFAULT}, {7, NONE},
    {8, DEFAULT}, {9, DEFAULT}, {10, DEFAULT}, {11, DEFAULT},
    {12, DEFAULT}, {13, DEFAULT}, {14, DEFAULT}, {15, NONE}
};

const double atiFTSensorROS::calibrationMatrixLFoot[6][6] = {
    {-0.74957,      0.24281,    -23.71506,  466.60093,  18.22998,   -468.17685},
    { 7.19002,      -546.10473, -6.38277,   266.54205,  -11.70446,  277.80902},
    {-695.34548,    -16.86802,  -637.23056, 11.25426,   -777.28613, 38.67756},
    {-0.77968,      -7.75893,   14.91040,   3.51086,    -16.01169,  4.68867},
    {-17.44361,     -0.36882,   8.83623,    -6.62325,    9.17076,   6.19479},
    {-0.31783,      7.67933,    -0.32218,   7.62097,    -0.22801,   7.95418}
};
const double atiFTSensorROS::calibrationMatrixRFoot[6][6] = {
    {3.66373, -2.53887,-56.93166, 472.72786, 55.48633,-480.00204},
    { 69.76364,-550.39726,-18.98287, 268.38284,-51.71793, 281.63366},
    {-694.83314,-48.95910,-677.24401,-53.91778,-721.44018,-44.23236},
    {0.81357, -7.81489, 14.99121,  5.16497,-16.16116,  2.94277},
    {-17.34220, -1.31783,  9.88078, -5.83100,  7.85218,  7.37678},
    {-0.78857,  7.80856, -1.07929,  7.63313, -0.74397,  7.78721}
};



int main(int argc, char** argv) {

    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nh;

    atiFTSensorROS ft(nh);

    ros::Rate r(SAMPLE_RATE);
    ROS_INFO("DAQ Initialize ...");
    for(int i=0; i<SAMPLE_RATE; i++)
        r.sleep();
    
    ROS_INFO("DAQ Initialize Done. Streaming started.");
    while(ros::ok())
    {
        ft.loop();
        r.sleep();

        ros::spinOnce();
    }
}
