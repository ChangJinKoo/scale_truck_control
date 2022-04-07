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

#define DATASIZE 80

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

typedef struct ZmqData{
	//Control center = 15, 20, CRC = 30, LRC = 10, 11, 12, LV = 0, FV1 = 1, FV2 = 2
	uint8_t src_index = 255;
	uint8_t tar_index = 255;
	
	//sensor failure
	bool alpha = false;
	bool beta = false;
	bool gamma = false;

	float cur_vel = 0.0f;
	float cur_dist = 0.0f;
	float cur_angle = 0.0f;
	float tar_vel = 0.0f;
	float tar_dist = 0.0f;
	float est_vel = 0.0f;  //estimated velocity
	float est_dist = 0.0f;

	//TM = 0, RCM = 1, GDM = 2
	uint8_t lrc_mode = 0;
	uint8_t crc_mode = 0;

	LaneCoef coef[3];
}ZmqData;

class ZMQ_CLASS{
public:
  explicit ZMQ_CLASS(ros::NodeHandle nh);
  ~ZMQ_CLASS();
  
  void* requestZMQ(ZmqData *send_data);
  void* replyZMQ(ZmqData *send_data);
  void* radioZMQ(ZmqData *send_data);
  void* dishZMQ();
  std::string getIPAddress();

  std::string zipcode_;
  std::string rad_group_, dsh_group_;
  std::string udp_ip_, tcpsub_ip_, tcppub_ip_, tcpreq_ip_, tcprep_ip_;

  bool controlDone_;
  bool rad_flag_, dsh_flag_, req_flag_, rep_flag_;
  ZmqData *rad_send_, *dsh_recv_, *req_send_, *req_recv_, *rep_send_, *rep_recv_;

private:
  ros::NodeHandle nodeHandle_;
  void init();
  bool readParameters();
  void* subscribeZMQ();
  void* publishZMQ();
  
  std::string interface_name_;
  zmq::context_t context_;
  zmq::socket_t sub_socket_, pub_socket_, req_socket_, rep_socket_, rad_socket_, dsh_socket_;
};
