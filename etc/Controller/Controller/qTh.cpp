#include "qTh.h"
#include "controller.h"

qTh::qTh(QObject *parent) : QThread(parent)
{

}

void qTh::run()
{
    ZmqData lv_tmp;
    ZmqData fv1_tmp;
    ZmqData fv2_tmp;

    while(1)
    {
        //Controller::mutex_.lock();
        lv_tmp = Controller::lv_data_;
        fv1_tmp = Controller::fv1_data_;
        fv2_tmp = Controller::fv2_data_;
        //Controller::mutex_.unlock();

        emit setValue(lv_tmp);
        emit setValue(fv1_tmp);
        emit setValue(fv2_tmp);

        msleep(100);
    }
}
