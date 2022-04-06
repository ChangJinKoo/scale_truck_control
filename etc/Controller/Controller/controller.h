#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QMainWindow>
#include <qTh.h>

#include <opencv2/opencv.hpp>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(QWidget *parent = nullptr);
    ~Controller();
    void sendData(int value_vel, int value_dist, int to);

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;
    int FV1_alpha;
    int FV1_beta;
    int FV1_gamma;
    int FV2_alpha;
    int FV2_beta;
    int FV2_gamma;

private slots:
    void on_MVelSlider_valueChanged(int value);

    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);

    void on_LVDistSlider_valueChanged(int value);

    void on_FV1VelSlider_valueChanged(int value);

    void on_FV1DistSlider_valueChanged(int value);

    void on_FV2VelSlider_valueChanged(int value);

    void on_FV2DistSlider_valueChanged(int value);

    void on_pushButton_clicked();

    void updateData(ZmqData zmq_data);

    void on_LVBox_activated(int index);

    void on_FV1Box_activated(int index);

    void on_FV2Box_activated(int index);

    void on_Send_clicked();

    void on_FV1_alpha_toggled(bool checked);

    void on_FV1_beta_toggled(bool checked);

    void on_FV1_gamma_toggled(bool checked);

    void on_FV2_alpha_toggled(bool checked);

    void on_FV2_beta_toggled(bool checked);

    void on_FV2_gamma_toggled(bool checked);

private:
    Ui::Controller *ui;
    ZMQ_CLASS ZMQ_SOCKET_;
    cv::Mat display_Map(ZmqData zmq_data);
    qTh* qthread;
};
#endif // CONTROLLER_H
