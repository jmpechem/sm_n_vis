
#include <sensoray826.h>


void sensoray826_dev::open()
{
    boardflags = S826_SystemOpen();

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        ROS_ERROR("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                ROS_WARN("board %d detected. try [ %d ] board \n", i, i);
            }
        }
    }
    else
    {
        for (int i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                ROS_INFO("board %d detected." , i);
            }
        }
        isOpen = true;
    }

    switch (errcode)
    {
    case S826_ERR_OK:           break;
    case S826_ERR_BOARD:        ROS_ERROR("Illegal board number"); break;
    case S826_ERR_VALUE:        ROS_ERROR("Illegal argument"); break;
    case S826_ERR_NOTREADY:     ROS_ERROR("Device not ready or timeout"); break;
    case S826_ERR_CANCELLED:    ROS_ERROR("Wait cancelled"); break;
    case S826_ERR_DRIVER:       ROS_ERROR("Driver call failed"); break;
    case S826_ERR_MISSEDTRIG:   ROS_ERROR("Missed adc trigger"); break;
    case S826_ERR_DUPADDR:      ROS_ERROR("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
    case S826_ERR_BOARDCLOSED:  ROS_ERROR("Board not open"); break;
    case S826_ERR_CREATEMUTEX:  ROS_ERROR("Can't create mutex"); break;
    case S826_ERR_MEMORYMAP:    ROS_ERROR("Can't map board"); break;
    default:                    ROS_ERROR("Unknown error"); break;
    }

}

void sensoray826_dev::analogSampleStart(const SLOTATTR *slotattrs, int count)
{
    for(int i=0; i<count; i++)
    {
        PRINT_ERR(S826_AdcSlotConfigWrite(board, i, slotattrs[i].chan, slotattrs[i].tsettle, S826_ADC_GAIN_1) );
    }
    PRINT_ERR( S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)   );  // enable all timeslots
    PRINT_ERR( S826_AdcTrigModeWrite(board, 0)                       );  // select continuous (untriggered) mode
    PRINT_ERR( S826_AdcEnableWrite(board, 1)                         );  // enable conversions

    isADCThreadOn = true;
    adcThread = boost::thread( boost::bind(&sensoray826_dev::analogSampleLoopThread, this) );
    //adcThread.start_thread();
}

void sensoray826_dev::analogSingleSamplePrepare(const SLOTATTR *slotattrs , int count)
{
    for(int i=0; i<count; i++)
    {
        PRINT_ERR(S826_AdcSlotConfigWrite(board, i, slotattrs[i].chan, slotattrs[i].tsettle, S826_ADC_GAIN_1) );
    }
    PRINT_ERR( S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)   );  // enable all timeslots
    PRINT_ERR( S826_AdcTrigModeWrite(board, 0)                       );  // select continuous (untriggered) mode
    PRINT_ERR( S826_AdcEnableWrite(board, 1)                         );  // enable conversions
}

void sensoray826_dev::analogSampleStop()
{
    if(isADCThreadOn)
    {
        isADCThreadOn = false;
        adcThread.join();
    }
    PRINT_ERR( S826_AdcEnableWrite(board, 0)                         );  // halt adc conversions
}

void sensoray826_dev::analogOversample()
{
    uint slotList = 0xFFFF;
    PRINT_ERR ( S826_AdcRead(board, _adBuf, _timeStamp,&slotList, 0));

    for(int i=0; i<ADC_MAX_SLOT; i++)
    {
        if ((((slotList >> (int)i) & 1) != 0)) {
            // extract adcdata, burstnum, and bufoverflowflag from buf
            adcDatas[i] = (int16_t)((_adBuf[i] & 0xFFFF));
            burstNum[i] = ((uint32_t)_adBuf[i] >> 24);
            adcVoltages[i] = adcDatas[i] * 10.0 / 32768;
        }
    }
    // ROS_INFO("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", adcVoltages[0], adcVoltages[1], adcVoltages[2], adcVoltages[3], adcVoltages[4], adcVoltages[5]);
}
void sensoray826_dev::analogSampleLoopThread()
{
    while(isADCThreadOn)
    {
        uint slotList = 0xFFFF;
        PRINT_ERR ( S826_AdcRead(board, _adBuf, _timeStamp,&slotList, 0));

        for(int i=0; i<ADC_MAX_SLOT; i++)
        {
            if ((((slotList >> (int)i) & 1) != 0)) {
                // extract adcdata, burstnum, and bufoverflowflag from buf
                adcDatas[i] = (int16_t)((_adBuf[i] & 0xFFFF));
                burstNum[i] = ((uint32_t)_adBuf[i] >> 24);
                adcVoltages[i] = adcDatas[i] * 10.0 / 32768;
            }
        }

        //ROS_INFO("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", adcVoltages[0], adcVoltages[1], adcVoltages[2], adcVoltages[3], adcVoltages[4], adcVoltages[5]);
        //ROS_INFO("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf ", axisData[0], axisData[1], axisData[2], axisData[3], axisData[4], axisData[5]);

        //ROS_INFO("%8x",buf[0]);
        //ROS_INFO("d= %d, B=%d",adcDatas[8],burstNum[8]);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}
