#include "rt_serial_port.h"

//void rt

// TODO: Reversed data.... please reverse......
static const struct rtser_config ser_config = {
    0xFFFF,                         /* config_mask */
    1500000,                        /* baud_rate */      // Why 1/2??? anyone who knows?
    RTSER_NO_PARITY,                /* parity */
    RTSER_8_BITS ,                  /* data_bits */
    RTSER_1_STOPB ,                 /* stop_bits */
    RTSER_NO_HAND ,                 /* handshake */
    RTSER_DEF_FIFO_DEPTH,           /* fifo_depth*/
    0,                              // reserved.. fake caution...
    RTSER_TIMEOUT_NONE,             /* rx_timeout */
    RTSER_DEF_TIMEOUT,              /* tx_timeout */
    1,      /* 0.000000001 s */     /* event_timeout */
    RTSER_RX_TIMESTAMP_HISTORY,     /* timestamp_history */
    RTSER_EVENT_RXPEND              /* event mask */
};


rt_serial_port::rt_serial_port() {
    DEBUG_PRINT = false;
    SocketFD = -1;
    SetPortName("");
}

rt_serial_port::rt_serial_port(const char* port_name) {
    DEBUG_PRINT = false;
    SocketFD = -1;
    SetPortName(port_name);
}

rt_serial_port::~rt_serial_port() {
    ClosePort();
}

void rt_serial_port::SetPortName(const char* port_name)
{
    strcpy(PortName, port_name);
}


bool rt_serial_port::OpenPort()
{
    SocketFD = rt_dev_open(PortName,0);
    if(SocketFD == -1)
    {
        printf("error on openning dev port : %s\n",PortName);
        return false;
    }

    int ret;
    ret = rt_dev_ioctl(SocketFD, RTSER_RTIOC_SET_CONFIG, &ser_config);
    if (ret)
    {
        printf("error on RTSER_RTIOC_WAIT_EVENT, %s\n",   strerror(-ret));
        return false;
    }
    return true;
}

void rt_serial_port::ClosePort()
{
    if(SocketFD != -1)
        rt_dev_close(SocketFD);
    SocketFD = -1;
}

int rt_serial_port::WritePort(unsigned char* packet, int packet_len)
{
    return rt_dev_write(SocketFD, packet, packet_len);
}

int rt_serial_port::ReadPort(unsigned char* packet, int packet_len)
{

    struct rtser_event rx_event = {0,};
    int err;

    err = rt_dev_ioctl(SocketFD, RTSER_RTIOC_WAIT_EVENT, &rx_event);

    if(rx_event.rx_pending == 0) return 0;
    //printf("%s : pending = %d, packet_len = %d\n",PortName,rx_event.rx_pending,packet_len);

    if(packet_len > rx_event.rx_pending)
        return rt_dev_read(SocketFD, packet, rx_event.rx_pending);

    return rt_dev_read(SocketFD, packet, packet_len);
}
