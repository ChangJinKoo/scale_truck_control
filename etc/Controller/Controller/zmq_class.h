#ifndef ZMQ_CLASS_H
#define ZMQ_CLASS_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <thread>
#include <chrono>
#include <mutex>

#include <zmq.hpp>

#define DATASIZE 80

typedef struct LaneCoef{
	float a = 0.0f;
	float b = 0.0f;
	float c = 0.0f;
}LaneCoef;

typedef struct ZmqData{
	//Control center = 20, CRC = 30, LRC = 10, 11, 12, LV = 0, FV1 = 1, FV2 = 2
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
  ZMQ_CLASS();
  ~ZMQ_CLASS();

  void* requestZMQ(ZmqData *send_data);
  std::string getIPAddress();

  std::string zipcode_;
  std::string tcpreq_ip0_, tcpreq_ip1_, tcpreq_ip2_;


  bool controlDone_;
  bool req_flag0_, req_flag1_, req_flag2_;
  ZmqData *req_recv0_, *req_recv1_, *req_recv2_;
  
private:
  void init();
  bool readParameters();
  
  std::string interface_name_;
  zmq::socket_t req_socket0_, req_socket1_, req_socket2_;
  zmq::context_t context_;
};

#endif
