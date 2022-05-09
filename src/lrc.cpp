#include "lrc/lrc.hpp"

#define PATH "/home/avees/catkin_ws/logfiles/"

using namespace std;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(ros::NodeHandle nh)
  : nodeHandle_(nh), ZMQ_SOCKET_(nh){
  
  init();  
}

LocalRC::~LocalRC(){
  is_node_running_ = false; 
  udpThread_.join();

  delete lrc_data_;
}

void LocalRC::init(){
  is_node_running_ = true;

  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string OcrSubTopicName;
  int OcrSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;
  std::string OcrPubTopicName;
  int OcrPubQueueSize;

  nodeHandle_.param("LrcParams/lrc_index", index_, 10);
  nodeHandle_.param("LrcParams/lrc_log_path", log_path_, std::string("/home/jetson/catkin_ws/logfiles/"));
  nodeHandle_.param("LrcParams/epsilon", epsilon_, 1.0f);
  nodeHandle_.param("LrcParams/lu_ob_A", a_, 0.6817f);
  nodeHandle_.param("LrcParams/lu_ob_B", b_, 0.3183f);
  nodeHandle_.param("LrcParams/lu_ob_L", l_, 0.2817f);
  nodeHandle_.param("LrcParams/enable_console_output", EnableConsoleOutput_, true);

  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("LrcSubPub/xavier_to_lrc/topic", XavSubTopicName, std::string("/xav2lrc_msg"));
  nodeHandle_.param("LrcSubPub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
  nodeHandle_.param("LrcSubPub/ocr_to_lrc/topic", OcrSubTopicName, std::string("/ocr2lrc_msg"));
  nodeHandle_.param("LrcSubPub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  nodeHandle_.param("LrcSubPub/lrc_to_xavier/topic", XavPubTopicName, std::string("/lrc2xav_msg"));
  nodeHandle_.param("LrcSubPub/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  nodeHandle_.param("LrcSubPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("/lrc2ocr_msg"));
  nodeHandle_.param("LrcSubPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */ 
  /************************/
  XavSubscriber_ = nodeHandle_.subscribe(XavSubTopicName, XavSubQueueSize, &LocalRC::XavCallback, this);
  OcrSubscriber_ = nodeHandle_.subscribe(OcrSubTopicName, OcrSubQueueSize, &LocalRC::OcrCallback, this);
//  OcrSubscriber_ = nodeHandle_.subscribe("/vel_msg", OcrSubQueueSize, &LocalRC::OcrCallback, this);

  /************************/
  /* ROS Topic Publisher */ 
  /************************/
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2xav>(XavPubTopicName, XavPubQueueSize);
  OcrPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);

  lrc_mode_ = 0;
  lrc_data_ = new ZmqData;
  lrc_data_->src_index = index_;
  lrc_data_->tar_index = 30;  //CRC

  lrcThread_ = std::thread(&LocalRC::communicate, this);
  tcpThread_ = std::thread(&LocalRC::request, this, lrc_data_);
  if (index_ == 10){
    udpThread_ = std::thread(&LocalRC::radio, this, lrc_data_);
  }
  else if (index_ == 11 || index_ == 12){
    udpThread_ = std::thread(&LocalRC::dish, this);
  }
}

bool LocalRC::isNodeRunning(){
  return is_node_running_;
}


void LocalRC::XavCallback(const scale_truck_control::xav2lrc &msg){
  std::scoped_lock lock(data_mutex_, time_mutex_);
  angle_degree_ = msg.steer_angle;
  cur_dist_ = msg.cur_dist;
  if(index_ == 10){  //only LV LRC
    tar_dist_ = msg.tar_dist;
    tar_vel_ = msg.tar_vel;
  }
  fi_encoder_ = msg.fi_encoder;
  beta_ = msg.beta;
  gamma_ = msg.gamma;
}

//void LocalRC::OcrCallback(const scale_truck_control::lrc2xav &msg){
void LocalRC::OcrCallback(const scale_truck_control::ocr2lrc &msg){
  std::scoped_lock lock(data_mutex_);
  cur_vel_ = msg.cur_vel;
  sat_vel_ = msg.u_k;  //saturated velocity
}

void LocalRC::rosPub(){
  scale_truck_control::lrc2xav xav;
  scale_truck_control::lrc2ocr ocr;
  { 
    std::scoped_lock lock(data_mutex_, time_mutex_);
    xav.alpha = alpha_;
    xav.cur_vel = cur_vel_;
    xav.tar_vel = tar_vel_;
    xav.tar_dist = tar_dist_;
    xav.lrc_mode = lrc_mode_;
    xav.crc_mode = crc_mode_;
    ocr.index = index_;
    ocr.steer_angle = angle_degree_;
    ocr.cur_dist = cur_dist_;
    ocr.tar_dist = tar_dist_;
    ocr.tar_vel = tar_vel_;
    ocr.est_vel = est_vel_;
    ocr.fi_encoder = fi_encoder_;
    ocr.alpha = alpha_;
  }
  XavPublisher_.publish(xav);
  OcrPublisher_.publish(ocr);
}

void LocalRC::radio(ZmqData* zmq_data)
{
  while(isNodeRunning()){
    {
      std::scoped_lock lock(data_mutex_);
      zmq_data->tar_vel = tar_vel_;
      zmq_data->tar_dist = tar_dist_;
    }
    ZMQ_SOCKET_.radioZMQ(zmq_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::dish()
{
  while(isNodeRunning()){
    ZMQ_SOCKET_.dishZMQ();
    updateData(ZMQ_SOCKET_.dsh_recv_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::request(ZmqData* zmq_data){
  struct timeval startTime, endTime;
  double diff_time = 0.0;
  while(isNodeRunning()){
    {
      std::scoped_lock lock(data_mutex_, time_mutex_);
      zmq_data->cur_vel = cur_vel_;
      zmq_data->tar_dist = tar_dist_; //because of CRC logging
      zmq_data->cur_dist = cur_dist_;
      zmq_data->alpha = alpha_;
      zmq_data->beta = beta_;
      zmq_data->gamma = gamma_;
      zmq_data->lrc_mode = lrc_mode_;
    }
    gettimeofday(&startTime, NULL);
    ZMQ_SOCKET_.requestZMQ(zmq_data);
    gettimeofday(&endTime, NULL);
    {
      std::scoped_lock lock(time_mutex_);
      req_time_ = ((endTime.tv_sec - startTime.tv_sec)* 1000.0) + ((endTime.tv_usec - startTime.tv_usec)/1000.0);
    }
    updateData(ZMQ_SOCKET_.req_recv_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::velSensorCheck(){
  std::scoped_lock lock(data_mutex_);
  if(!fi_encoder_){
    hat_vel_ = a_ * hat_vel_ + b_ * sat_vel_ - l_ * (cur_vel_ - hat_vel_);
    if(fabs(cur_vel_ - hat_vel_) > epsilon_){
      alpha_ = true;
    }
  }
  else{
    hat_vel_ = a_ * hat_vel_ + b_ * 2.0f - l_ * (0.0f - hat_vel_);
    if(fabs(0.0f - hat_vel_) > epsilon_){
      alpha_ = true;
    }
  }
/*  
  else{  //Recovery
    alpha_ = false;
  }
*/
}

void LocalRC::updateMode(uint8_t crc_mode){
  std::scoped_lock lock(data_mutex_);
  if(index_ == 10){  //LV
    if(beta_ || crc_mode == 2){  //Camera sensor failure
      lrc_mode_ = 2;  //GDM
    }
    else if((alpha_ || gamma_) && (crc_mode == 0 || crc_mode == 1)){
      lrc_mode_ = 1;  //RCM
    }
    else if(crc_mode == 0 || (!alpha_ && !beta_ && !gamma_)){
      lrc_mode_ = 0;  //TM
    }
  }
  else{  //FV1, FV2
    if((beta_ && gamma_) || crc_mode == 2){
      lrc_mode_ = 2;  
    }
    else if((alpha_ || beta_ || gamma_) && (crc_mode == 0 || crc_mode == 1)){
      lrc_mode_ = 1;  
    }
    else if(crc_mode == 0 || (!alpha_ && !beta_ && !gamma_)){
      lrc_mode_ = 0;
    }
  }
}

void LocalRC::updateData(ZmqData* zmq_data){
  std::scoped_lock lock(data_mutex_);
  if(zmq_data->src_index == 30){  //from CRC
    est_vel_ = zmq_data->est_vel;
    crc_mode_ = zmq_data->crc_mode;
  }
  else if(zmq_data->src_index == 10){  //from LV LRC to FVs LRC
    tar_vel_ = zmq_data->tar_vel;
    tar_dist_ = zmq_data->tar_dist;
  }
}


void LocalRC::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "LRC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  ifstream read_file;
  ofstream write_file;
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[7] = i/10 + '0';  //ASCII
      file_name[8] = i%10 + '0';
      sprintf(file, "%s%s", log_path_.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "Time,Request_time,Tar_dist,Cur_dist,Tar_vel,Cur_vel,Est_vel,Sat_vel,Hat_vel,Alpha" << endl; //seconds
    flag = true;
  }
  else{
    std::scoped_lock lock(data_mutex_, time_mutex_);
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, req_time_, tar_dist_, cur_dist_, tar_vel_, cur_vel_, est_vel_, sat_vel_, hat_vel_, alpha_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << endl;
  }
  write_file.close();
}

void LocalRC::printStatus(){
  static int cnt = 0;
  static int CNT = 0;
  if (EnableConsoleOutput_){
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nPredict Velocity:\t%.3f", est_vel_);
    printf("\nTarget Velocity:\t%.3f", tar_vel_);
    printf("\nCurrent Velocity:\t%.3f", cur_vel_);
    printf("\nTarget Distance:\t%.3f", tar_dist_);
    printf("\nCurrent Distance:\t%.3f", cur_dist_);
    printf("\nSaturated Velocity:\t%.3f", sat_vel_);
    printf("\nEstimated Value:\t%.3f", fabs(cur_vel_ - hat_vel_));
    printf("\nalpha, beta, gamma:\t%d / %d / %d", alpha_, beta_, gamma_); 
    printf("\nCRC mode, LRC mode:\t%d / %d", crc_mode_, lrc_mode_);
    printf("\n");
  }
}

void LocalRC::communicate(){
  struct timeval startTime, endTime;
  gettimeofday(&startTime, NULL);
  static int cnt = 0;
  while(ros::ok()){
    velSensorCheck();
    updateMode(crc_mode_);
    rosPub();
    printStatus();
    recordData(&startTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    if(!isNodeRunning()){
      ros::requestShutdown();
      break;
    }
  }
}

}
