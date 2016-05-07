
#ifndef RT_DYNAMIXEL_PRO_H_
#define RT_DYNAMIXEL_PRO_H_

#include <stdio.h>
#include <string.h>
#include <rtdk.h>
#include <rtdm/rtserial.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <vector>
#include "rt_serial_port.h"


#define MAXNUM_TXPACKET     (500)
#define MAXNUM_RXPACKET     (500)
#define BROADCAST_ID		(0xFE)

#define LATENCY_TIME		(0)		// ms (USB2Serial Latency timer)

#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))


void dxl_control(void* parent);

namespace DXL_PRO {
    enum {
        PKT_HEADER0,
        PKT_HEADER1,
        PKT_HEADER2,
        PKT_RESERVED,
        PKT_ID,
        PKT_LENGTH_L,
        PKT_LENGTH_H,
        PKT_INSTRUCTION,
        PKT_PARAMETER
    };

    enum {
        COMM_TXSUCCESS,
        COMM_RXSUCCESS,
        COMM_TXFAIL,
        COMM_RXFAIL,
        COMM_TXERROR,
        COMM_RXWAITING,
        COMM_RXTIMEOUT,
        COMM_RXCORRUPT
    };

    enum {
        INST_PING			= 1,
        INST_READ			= 2,
        INST_WRITE			= 3,
        INST_REG_WRITE		= 4,
        INST_ACTION			= 5,
        INST_FACTORY_RESET	= 6,
        INST_REBOOT			= 8,
        INST_SYSTEM_WRITE	= 13,	// 0x0D
        INST_STATUS			= 85,	// 0x55
        INST_SYNC_READ		= 130,	// 0x82
        INST_SYNC_WRITE		= 131,	// 0x83
        INST_BULK_READ		= 146,	// 0x92
        INST_BULK_WRITE		= 147	// 0x93
    };

    enum {
        ERRBIT_VOLTAGE		= 1,
        ERRBIT_ANGLE		= 2,
        ERRBIT_OVERHEAT		= 4,
        ERRBIT_RANGE		= 8,
        ERRBIT_CHECKSUM		= 16,
        ERRBIT_OVERLOAD		= 32,
        ERRBIT_INSTRUCTION	= 64
    };

    class PingInfo
    {
    public:
        int ID;
        int ModelNumber;
        int FirmwareVersion;

        PingInfo() : ID(-1), ModelNumber(-1), FirmwareVersion(-1) { }
    };

    class BulkReadData
    {
    public:
        int             iID;
        int             iStartAddr;
        int             iLength;
        int             iError;
        unsigned char*  pucTable;

        BulkReadData()
        {
            iID = 0; iStartAddr = 0; iLength = 0; iError = 0;
            pucTable = 0;

        }
        ~BulkReadData()
        {
//			if(pucTable != 0)
//				delete[] pucTable;
        }
    };


    struct SyncReadData {
        int				iID;
        int				iError;
        unsigned char	pucTable[200]; // pre-allocate for RTOS
    };

    class rt_dynamixel
    {
    private:
        double PacketStartTime;
        double PacketWaitTime;
        double ByteTransferTime;

        int GetBaudrate(int baud_num);

        virtual bool IsPacketTimeout();

        unsigned short UpdateCRC(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
        void AddStuffing(unsigned char *packet);
        void RemoveStuffing(unsigned char *packet);

        int TxPacket(unsigned char *txpacket);
        int RxPacket(unsigned char *rxpacket);
        int TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int *error);

    public:
        rt_serial_port *ComPort;

        rt_dynamixel();
        rt_dynamixel(const char* port_name);
        virtual ~rt_dynamixel();

        bool Connect();
        void Disconnect();
        bool SetBaudrate(int baud);

        int Ping(int id, int *error);
        int Ping(int id, PingInfo *info, int *error);
        int BroadcastPing(std::vector<PingInfo>& vec_info);
        int Reboot(int id, int *error);
        int FactoryReset(int id, int option, int *error);

        int Read(int id, int address, int length, unsigned char* data, int *error);
        int ReadByte(int id, int address, int *value, int *error);
        int ReadWord(int id, int address, int *value, int *error);
        int ReadDWord(int id, int address, long *value, int *error);

        int Write(int id, int address, int length, unsigned char* data, int *error);
        int WriteByte(int id, int address, int value, int *error);
        int WriteWord(int id, int address, int value, int *error);
        int WriteDWord(int id, int address, long value, int *error);

        int SyncWrite(int start_addr, int data_length, unsigned char* param, int param_length);
        int SyncRead(int start_addr, int data_length, unsigned char id_length, unsigned char *read_ids, SyncReadData* rxdata, unsigned char *received);
        //void SyncDataFree(unsigned char id_length, PSyncData rxdata);

        int BulkRead(std::vector<BulkReadData>& data);
    };



    // Dynamixel Custom Functions and Classes
    // Author : Park Suhan (psh117@gmail.com)

    const int H54_180_DEG = 250950; // H54 Resolution per 180 degree
    const int H42_180_DEG = 151900; // H42 Resolution per 180 degree

    const double H54_GEAR_RATIO = 502.0;
    const double H42_GEAR_RATIO = 304.0;

    const double PI = 3.1415926535897932384626433832795;
    const double DEG_TO_H54 = H54_180_DEG / 180.0;
    const double DEG_TO_H42 = H42_180_DEG / 180.0;
    const double RAD_TO_H54 = H54_180_DEG / PI;
    const double RAD_TO_H42 = H42_180_DEG / PI;

    const double H54_TO_DEG = 180.0 / H54_180_DEG;
    const double H42_TO_DEG = 180.0 / H42_180_DEG;
    const double H54_TO_RAD = PI / H54_180_DEG;
    const double H42_TO_RAD = PI / H42_180_DEG;

    // control period extern
    //const double

    enum dxl_pro_type { H54 = 1, H42 = 2};

    struct dxl_inverse  ///< For inversed access
    {
        int channel;
        int index;
    };

    struct dxl_gains
    {
        int id;
        int position_p_gain;
        int velocity_p_gain;
        int velocity_i_gain;
    };

    struct dxl_pro_data
    {
        uint8_t id;
        dxl_pro_type type;
        int32_t position;
        int32_t velocity;
        int16_t current;

        double aim_radian;

        const double position_rad()
        {
            if (type == H54) return position * H54_TO_RAD;
            if (type == H42) return position * H42_TO_RAD;
        }
        const double position_deg()
        {
            if (type == H54) return position * H54_TO_DEG;
            if (type == H42) return position * H42_TO_DEG;
        }
        double velocity_radsec()
        {
            if (type == H54) return (velocity / H54_GEAR_RATIO) * PI * 2 / 60;
            if (type == H42) return (velocity / H42_GEAR_RATIO) * PI * 2 / 60;
        }
        double velocity_degsec()
        {
            if (type == H54) return (velocity / H54_GEAR_RATIO) * 360 / 60;
            if (type == H42) return (velocity / H42_GEAR_RATIO) * 360 / 60;
        }
        double velocity_rpm()
        {
            if (type == H54) return velocity / H54_GEAR_RATIO;
            if (type == H42) return velocity / H42_GEAR_RATIO;
        }
        double current_amp()
        {
            if (type == H54) return current * 33.0 / 2048;
            if (type == H42) return current * 8.250 / 2048;
        }
        double current_mA()
        {
            if (type == H54) return current * 33000.0 / 2048;
            if (type == H42) return current * 8250.0 / 2048;
        }
        double torque()
        {
        }
    };


    /**
     * @brief The RTDynamixelPro class for Linux based real-time system.
     * @n It contains rt task for real-time control.
     *
     */
    class RTDynamixelPro : public rt_dynamixel
    {
    private:
        SyncReadData pSyncData[253];    ///< SyncRead resource
        unsigned char pucIDList[253];   ///< Motor ID Datas
        int nMotorNum;                  ///< The number of motos

        // bool bExitThread;               ///< For rt_task exit

        // RTOS
        RT_TASK rttTaskObject;          ///< Real-time task object

        bool checkControlLoopEnabled(const char *szSetName);

    public:
        RT_MUTEX rttDataMutex;           ///< Real-time mutex for data exchange
        RTIME rttLoopStartTime;         ///< Control loop start time.
        RTIME rttLoopTimeoutTime;       ///< Control loop timeout time.
        int nIndex;                     ///< Serial channel index
        bool bControlLoopEnable;        ///< For enable control loop
        bool bControlLoopProcessing;    ///< for safe

        dxl_pro_data vMotorData[10];
        //std::vector<dxl_pro_data> vMotorData;   ///< Data of all motors

        RTDynamixelPro(int index) :
            rt_dynamixel(), nIndex(index), bControlLoopEnable(false)
        {
            char threadName[40] = {0, };
            char threadName2[40] = {0, };
            char mutexName[40] = {0, };
            sprintf(threadName,"dxl ctr thr %d", index);
            sprintf(mutexName,"dxl dat mtx %d", index);
            rt_mutex_create(&rttDataMutex, mutexName);
            rt_task_create(&rttTaskObject, threadName, 0, 50, 0);
        }

        ~RTDynamixelPro()
        {
            rt_task_delete(&rttTaskObject);
            rt_mutex_delete(&rttDataMutex);
        }
        void mutex_acquire()
        { rt_mutex_acquire(&rttDataMutex,TM_INFINITE); }
        void mutex_release()
        { rt_mutex_release(&rttDataMutex); }

        dxl_pro_data& operator [] (const int& i) { return vMotorData[i]; }

        bool IsPacketTimeout()
        {
            if (rt_timer_read() > rttLoopTimeoutTime)
            { return true; }
            return false;
        }


        void startThread()      ///< Enable thread. @warning You must set ID List before calling this function.
        { rt_task_start(&rttTaskObject, &dxl_control, (void*)this); }

        int getMotorNum() { return nMotorNum; }


        
        void setIDList(int motorNum, dxl_pro_data *motorList);      ///< Set ID List for whole control.

        // non control loop function
        int setHomingOffset(int index, int nValue, int* error);
        // non control loop function
        int setVelocityGain(int index, int nVelocityIGain, int nVelocityPGain, int* error);
        int setPositionGain(int index, int nPositionPGain, int* error);

        void setReturnDelayTime(int nValue);
        void setAllTorque(int nValue);
        void setLEDs(int nRedValue, int nGreenValue, int nBlueValue);
        void setPIGains(int nVelocityIGain, int nVelocityPGain, int nPositionPGain);

        void setAllAcceleration(int nValue);
        void setAllVelocity(int nValue);

        // control loop function
        void setAllDegree(double dDegree);
        void setEachDegree(double *pdDegrees);

        void setAllRadian(double dRadian);
        void setEachRadian(double *pdRadians);

        void printErrorCode(int id, int errorCode);

        int getAllStatus();
    };

}

#endif // RT_SERIAL_PORT_H
