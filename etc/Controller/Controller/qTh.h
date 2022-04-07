#ifndef QTH_H
#define QTH_H

#include <QThread>

#include "zmq_class.h"

class Controller;

class qTh : public QThread
{
    Q_OBJECT
public:
    explicit qTh(QObject* parent = 0);
    int recvInit();

private:
    void run();
signals:
    void setValue(ZmqData tmp);
};


#endif // QTH_H
