
#ifndef RT_SERIAL_PORT_H
#define RT_SERIAL_PORT_H

#include <stdio.h>
#include <string.h>
#include <rtdk.h>
#include <rtdm/rtserial.h>
#include <native/mutex.h>

class rt_serial_port
{
private:
    char PortName[20];

    int GetBaud(int baud);
    bool SetupSerialPort(int baud);
    bool SetBaudDevisor(int speed);

public:
    int SocketFD;
    static const int DEFAULT_BAUDRATE = 57600;
    bool DEBUG_PRINT;

    rt_serial_port();
    rt_serial_port(const char* port_name);
    virtual ~rt_serial_port();


    bool OpenPort();
    void ClosePort();
    void SetPortName(const char* port_name);

    int WritePort(unsigned char* packet, int packet_len);
    int ReadPort(unsigned char* packet, int packet_len);

};

#endif // RT_SERIAL_PORT_H
