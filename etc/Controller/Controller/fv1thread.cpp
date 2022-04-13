#include "fv1thread.h"
#include "controller.h"

FV1Thread::FV1Thread(QObject *parent) : QThread(parent)
{

}

void FV1Thread::run()
{
    ZmqData fv1_tmp;
    fv1_tmp.src_index = 1;

    while(1)
    {
        emit request(fv1_tmp);

        Controller::fv1_mutex_.lock();
        fv1_tmp = Controller::fv1_data_;
        Controller::fv1_mutex_.unlock();

        emit setValue(fv1_tmp);

        msleep(20);
    }
}
