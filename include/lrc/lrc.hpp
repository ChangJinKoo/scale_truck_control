#pragma once

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include <string>

#include "zmq_class/zmq_class.h"

#include <scale_truck_control/xav2lrc.h>
#include <scale_truck_control/ocr2lrc.h>
#include <scale_truck_control/lrc2xav.h>
#include <scale_truck_control/lrc2ocr.h>

using namespace std;

namespace LocalResiliencyCoordinator{

class LocalRC{
  public:
    LocalRC(ros::NodeHandle nh);
    ~LocalRC();

    void communicate();

  private:
    ZMQ_CLASS ZMQ_SOCKET_;

    ros::NodeHandle nodeHandle_;
    ros::Subscriber XavSubscriber_;  
    ros::Subscriber OcrSubscriber_;  
    ros::Publisher XavPublisher_;
    ros::Publisher OcrPublisher_;

    int index_;
    ZmqData* lrc_data_;
    uint8_t lrc_mode_;
    uint8_t crc_mode_;

    void init();
    bool isNodeRunning();
    void XavCallback(const scale_truck_control::xav2lrc &msg);
    void OcrCallback(const scale_truck_control::ocr2lrc &msg);
    void rosPub();
    void radio(ZmqData* zmq_data);
    void dish();
    void request(ZmqData* zmq_data);
    void velSensorCheck();
    void updateMode(uint8_t crc_mode);
    void updateData(ZmqData* zmq_data);
    void recordData(struct timeval *startTime);
    void printStatus();

    bool is_node_running_;
    bool EnableConsoleOutput_;
    std::string log_path_;
    float a_, b_, l_;
    float epsilon_;
    bool alpha_ = false;
    bool beta_ = false;
    bool gamma_ = false;

    float angle_degree_;
    float cur_dist_;
    float tar_dist_;
    float cur_vel_ = 0;
    float tar_vel_ = 0;
    float est_vel_ = 0;
    float hat_vel_ = 0;
    float sat_vel_ = 0;
    double time_ = 0;

    std::thread lrcThread_;
    std::thread udpThread_;
    std::thread tcpThread_;
    std::mutex data_mutex_;
};

}
