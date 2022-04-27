#include <fstream>
#include "zmq_class.h"

namespace CentralResiliencyCoordinator{

class CentralRC{
  public:
    CentralRC();
    ~CentralRC();

    struct timeval launch_time_;
    void communicate();
    bool is_node_running_;

  private:
    ZMQ_CLASS ZMQ_SOCKET_;

    void init();
    void reply(ZmqData* zmq_data);
    void estimateVelocity(uint8_t index);
    void modeCheck(uint8_t lv_mode, uint8_t fv1_mode, uint8_t fv2_mode);
    void recordData(struct timeval *time);
    void printStatus();
    void updateData(ZmqData* zmq_data);

    bool time_flag_;
    uint8_t index_;
    uint8_t crc_mode_;

    ZmqData *lv_data_, *fv1_data_, *fv2_data_;

    float fv1_prev_dist_, fv2_prev_dist_;
    float lv_est_vel_, fv1_est_vel_, fv2_est_vel_;
    float sampling_time_;
    std::string log_path_ = "/home/avees/logfiles/";

    struct timeval start_time_, end_time_;
    double time_;

    std::thread repThread0_, repThread1_, repThread2_;
    std::mutex data_mutex_;
};

}

