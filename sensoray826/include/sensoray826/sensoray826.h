
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "826api.h"

#define PRINT_ERR(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { ROS_INFO("\nERROR: %d\n", errcode); }


struct SLOTATTR
{
    uint chan;      // analog input channel
    uint tsettle;   // settling time in microseconds
};

class sensoray826_dev
{

    static const int ADC_MAX_SLOT = 16;

    uint board;// change this if you want to use other than board number 0
    int errcode;
    int boardflags;        // open 826 driver and find all 826 boards
    bool isOpen;

    bool isADCThreadOn;
    boost::thread adcThread;


    uint _timeStamp[ADC_MAX_SLOT];
    int _adBuf[ADC_MAX_SLOT];



public:
    // Analog Datas
    int adcDatas[ADC_MAX_SLOT];
    double adcVoltages[ADC_MAX_SLOT];
    int burstNum[ADC_MAX_SLOT];



public:
    sensoray826_dev(int boardno = 0) : board(boardno), errcode(S826_ERR_OK), isOpen(false), isADCThreadOn(false) {}
    virtual ~sensoray826_dev() { analogSampleStop(); S826_SystemClose(); }

    void setBoard(uint boardno)   { board = boardno; }
    void open();

    /**
     * @brief analogSampleStart, Preparing ADC and turning on the AD thread
     * @param slotattrs ADC channel slots
     * @param count number of adc channel
     */
    void analogSampleStart(const SLOTATTR *slotattrs , int count);


    /**
     * @brief analogSingleSamplePrepare, Preparing ADC but not turning on the AD thread
     * @param slotattrs ADC channel slots
     * @param count number of adc channel
     */
    void analogSingleSamplePrepare(const SLOTATTR *slotattrs , int count);

    /**
     * @brief analogSampleStop Stop current AD thread and ADC
     */
    void analogSampleStop();

    /**
     * @brief analogOversample Do a single sample
     * @warning before calling this method, you should check whether ADC is prepared.
     */
    void analogOversample();

    /**
     * @brief analogSampleLoopThread
     * AD Thread, turned on by analogSampleStart(..., ...)
     */
    void analogSampleLoopThread();

    double lowPassFilter(double input, double prev, double ts, double tau)
    {
        return (tau*prev + ts*input)/(tau+ts);
    }


};
