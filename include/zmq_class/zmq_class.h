#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <thread>
#include <chrono>
#include <mutex>

#include <zmq.hpp>

#include <ros/ros.h>

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

typedef struct ZmqData{
	//Control center = 10, CRC = 11, LRC = 22, LV = 0, FV1 = 1, FV2 = 2
	uint8_t src_num = 255;
	
	//sensor failure
	bool alpha = false;
	bool beta = false;
	bool gamma = false;

	float cur_vel = 0.0f;
	float cur_dist = 0.0f;
	float tar_vel = 0.0f;
	float tar_dist = 0.0f;
	float est_vel = 0.0f;  //estimated velocity

	//TM = 0, RCM = 1, GDM = 2
	uint8_t lrc_mode = 0;
	uint8_t crc_mode = 0;

	LaneCoef lc;
}ZmqData;

class ZMQ_CLASS{
public:
  explicit ZMQ_CLASS(ros::NodeHandle nh);
  ~ZMQ_CLASS();
  
  std::string getIPAddress();

  std::string zipcode_;
  std::string rad_group_, dsh_group_;
  std::string udp_ip_, tcpsub_ip_, tcppub_ip_, tcpreq_ip_, tcprep_ip_;

  bool controlDone_;
  std::string send_req_, recv_req_, send_rep_, recv_rep_, recv_sub_, send_pub_, send_rad_, recv_dsh_;
  ZmqData* zmq_data_;
  
private:
  ros::NodeHandle nodeHandle_;
  void init();
  bool readParameters();
  void* subscribeZMQ();
  void* publishZMQ();
  void* requestZMQ();
  void* replyZMQ();
  void* radioZMQ();
  void* dishZMQ();
  void* clientZMQ();
  void* serverZMQ();
  
  std::string interface_name_;
  std::thread subThread_, pubThread_, reqThread_, repThread_, radThread_, dshThread_;
  zmq::context_t context_;
  zmq::socket_t sub_socket_, pub_socket_, req_socket_, rep_socket_, rad_socket_, dsh_socket_;
  bool sub_flag_, pub_flag_, rad_flag_, dsh_flag_, req_flag_, rep_flag_;
};