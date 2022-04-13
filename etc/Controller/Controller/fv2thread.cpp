#include "fv2thread.h"
#include "controller.h"

FV2Thread::FV2Thread(QObject *parent) : QThread(parent)
{

}

void FV2Thread::run()
{
    ZmqData fv2_tmp;
    fv2_tmp.src_index = 2;

    while(1)
    {
        emit request(fv2_tmp);

        Controller::fv2_mutex_.lock();
        fv2_tmp = Controller::fv2_data_;
        Controller::fv2_mutex_.unlock();

        emit setValue(fv2_tmp);

        msleep(100);
    }
}
