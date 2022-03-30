#include "includes/crc.hpp"

namespace CentralResiliencyCoordinator{
  
CentralRC::CentralRC()
  : ZMQ_SOCKET_(){

  init();  
}

CentralRC::~CentralRC(){
  delete lv_data_;
  delete fv1_data_;
  delete fv2_data_;
}

void CentralRC::init(){
  index_ = 30;
  crc_mode_ = 0;

  lv_data_ = new ZmqData;
  fv1_data_ = new ZmqData;
  fv2_data_ = new ZmqData;

  lv_data_->src_index = index_;
  lv_data_->tar_index = 10;

  fv1_data_->src_index = index_;
  fv1_data_->tar_index = 11;

  fv2_data_->src_index = index_;
  fv2_data_->tar_index = 12;

  fv1_prev_dist_ = 0.8f;
  fv2_prev_dist_ = 0.8f;

  time_flag_ = false;
  sampling_time_ = 0.1;
}

void CentralRC::reply(ZmqData* zmq_data){
  if(zmq_data->tar_index == 10){  //LV
    zmq_data->est_vel = lv_est_vel_;
    zmq_data->crc_mode = crc_mode_;
  }
  else if(zmq_data->tar_index == 11){  //FV1
    zmq_data->est_vel = fv1_est_vel_;
    zmq_data->crc_mode = crc_mode_;
  }
  else if(zmq_data->tar_index == 12){  //FV2
    zmq_data->est_vel = fv2_est_vel_;
    zmq_data->crc_mode = crc_mode_;
  }
  ZMQ_SOCKET_.replyZMQ(zmq_data);
}

void CentralRC::estimateVelocity(uint8_t index){
  assert(index < 3);

  if (index == 10){  //LV
    if (!lv_data_->alpha){
      lv_data_->est_vel = lv_data_->cur_vel;
    }
    else if (lv_data_->alpha && !lv_data_->alpha){
      lv_data_->est_vel = ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time_) + fv1_data_->cur_vel;
    }
    else if ((lv_data_->alpha || fv1_data_->alpha) && !fv2_data_->alpha){
      lv_data_->est_vel = ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time_) + ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time_) + fv2_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail 
      lv_data_->est_vel = 0;
      crc_mode_ = 2;
    }
  }
  else if (index == 11){  //FV1
    if (!fv1_data_->alpha){
      fv1_data_->est_vel = fv1_data_->cur_vel;
    }
    else if (fv1_data_->alpha && !lv_data_->alpha){
      fv1_data_->est_vel = ((-1.0f) * ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time_)) + lv_data_->cur_vel;
    }
    else if ((fv1_data_->alpha || lv_data_->alpha) && !fv2_data_->alpha){
      fv1_data_->est_vel = ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time_) + fv2_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail
      fv1_data_->est_vel = 0;
      crc_mode_ = 2;
    }
  }
  else if (index == 12){  //FV2
    if (!fv2_data_->alpha){
      fv2_data_->est_vel = fv2_data_->cur_vel;
    }
    else if (fv2_data_->alpha && !fv1_data_->alpha){
      fv2_data_->est_vel = ((-1.0f) * ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time_)) + fv1_data_->cur_vel;
    }
    else if ((fv2_data_->alpha || fv1_data_->alpha) && !lv_data_->alpha){
      fv2_data_->est_vel = ((-1.0f) * ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time_)) + ((-1.0f) * ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time_)) + lv_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail
      fv2_data_->est_vel = 0;
      crc_mode_ = 2;
    }
  }
}

void CentralRC::modeCheck(uint8_t lv_mode, uint8_t fv1_mode, uint8_t fv2_mode){
  if ((lv_mode == 0) && (fv1_mode == 0) && (fv2_mode == 0)){
    crc_mode_ = 0;
  }
  else if (((lv_mode == 2) || (fv1_mode == 2) || (fv2_mode == 2)) || ((lv_mode == 1) && (fv1_mode == 1) && (fv2_mode == 1))){
    crc_mode_ = 2;
  }
  else{
    crc_mode_ = 1;
  }
}

void CentralRC::printStatus(){
  printf("\033[2J\033[1;1H");
  printf("CRC is running ...\n");
  printf("CRC and each MODEs of LV, FV1, FV2:\t%d || %d, %d, %d\n", crc_mode_, lv_data_->lrc_mode, fv1_data_->lrc_mode, fv2_data_->lrc_mode);
  printf("Predict Velocitys of LV, FV1, FV2:\t%.3f, %.3f, %.3f\n", lv_data_->est_vel, fv1_data_->est_vel, fv2_data_->est_vel);
  printf("Sampling_time_:\t%.3f\n", sampling_time_);
  printf("LV current distance:\t%.3f\n", lv_data_->cur_dist);
}

void CentralRC::updateData(ZmqData* zmq_data){
  if(zmq_data->tar_index == 30){
    if(zmq_data->src_index == 10){
      lv_data_->cur_vel = zmq_data->cur_vel;
      lv_data_->cur_dist = zmq_data->cur_dist;
      lv_data_->alpha = zmq_data->alpha;
      lv_data_->beta = zmq_data->beta;
      lv_data_->gamma = zmq_data->gamma;
      lv_data_->lrc_mode = zmq_data->lrc_mode;
    }
    else if(zmq_data->src_index == 11){
      fv1_data_->cur_vel = zmq_data->cur_vel;
      fv1_data_->cur_dist = zmq_data->cur_dist;
      fv1_data_->alpha = zmq_data->alpha;
      fv1_data_->beta = zmq_data->beta;
      fv1_data_->gamma = zmq_data->gamma;
      fv1_data_->lrc_mode = zmq_data->lrc_mode;
    }
    else if(zmq_data->src_index == 12){
      fv2_data_->cur_vel = zmq_data->cur_vel;
      fv2_data_->cur_dist = zmq_data->cur_dist;
      fv2_data_->alpha = zmq_data->alpha;
      fv2_data_->beta = zmq_data->beta;
      fv2_data_->gamma = zmq_data->gamma;
      fv2_data_->lrc_mode = zmq_data->lrc_mode;
    }
  }
}

void CentralRC::communicate(){  
  modeCheck(lv_data_->lrc_mode, fv1_data_->lrc_mode, fv2_data_->lrc_mode);
 
  repThread0_ = std::thread(&CentralRC::reply, this, lv_data_);
  repThread1_ = std::thread(&CentralRC::reply, this, fv1_data_);
  repThread2_ = std::thread(&CentralRC::reply, this, fv2_data_);
  repThread0_.join();
  repThread1_.join();
  repThread2_.join();
  updateData(ZMQ_SOCKET_.rep_recv0_);
  updateData(ZMQ_SOCKET_.rep_recv1_);
  updateData(ZMQ_SOCKET_.rep_recv2_);

  if(time_flag_){
    gettimeofday(&end_time_, NULL);
    sampling_time_ = (end_time_.tv_sec - start_time_.tv_sec) + ((end_time_.tv_usec - start_time_.tv_usec)/1000000.0);  //seconds
  }

  estimateVelocity(0);
  estimateVelocity(1);
  estimateVelocity(2);

  repThread0_ = std::thread(&CentralRC::reply, this, lv_data_);
  repThread1_ = std::thread(&CentralRC::reply, this, fv1_data_);
  repThread2_ = std::thread(&CentralRC::reply, this, fv2_data_);
  repThread0_.join();
  repThread1_.join();
  repThread2_.join();
  updateData(ZMQ_SOCKET_.rep_recv0_);
  updateData(ZMQ_SOCKET_.rep_recv1_);
  updateData(ZMQ_SOCKET_.rep_recv2_);
  
  gettimeofday(&start_time_, NULL);
  if(!time_flag_) time_flag_ = true;

  fv1_prev_dist_ = fv1_data_->cur_dist;
  fv2_prev_dist_ = fv2_data_->cur_dist;

  printStatus();
}

}
