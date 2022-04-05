#include "qTh.h"

qTh::qTh(QObject *parent) : QThread(parent)
{

}

void qTh::run()
{
    //UDPrecv.GROUP_ = "239.255.255.250";
    //UDPrecv.PORT_ = 9307;
    //UDPrecv.recvInit();
    //UDPsock::UDP_DATA tmp;
    ZmqData tmp;

    while(1)
    {
        //UDPrecv.recvData(&tmp);
        emit setValue(tmp);
        msleep(1);
    }
}
