#include "lvthread.h"
#include "controller.h"

LVThread::LVThread(QObject *parent) : QThread(parent)
{

}

void LVThread::run()
{
    ZmqData lv_tmp;
    lv_tmp.src_index = 0;

    while(1)
    {
        emit request(lv_tmp);

        Controller::lv_mutex_.lock();
        lv_tmp = Controller::lv_data_;
        Controller::lv_mutex_.unlock();

        emit setValue(lv_tmp);

        msleep(20);
    }
}
