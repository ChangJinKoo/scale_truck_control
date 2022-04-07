/*
 * ScaleTruckController.h
 *
 *  Created on: June 2, 2020
 *      Author: Hyeongyu Lee
 *   Institute: KMU, Avees Lab
 */

#pragma once

//C++
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <string>
#include <condition_variable>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <obstacle_detector/Obstacles.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

#include "lane_detect/lane_detect.hpp"
#include "zmq_class/zmq_class.h"

//custom msgs
#include <scale_truck_control/lrc2xav.h>
#include <scale_truck_control/xav2lrc.h>

namespace scale_truck_control {

class ScaleTruckController {
  public:
    explicit ScaleTruckController(ros::NodeHandle nh);

    ~ScaleTruckController();

  private:
    bool readParameters();

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void objectCallback(const obstacle_detector::Obstacles &msg);
    void XavSubCallback(const scale_truck_control::lrc2xav &msg);
    //bool publishControlMsg(const scale_truck_control::ctl msg);

    ros::NodeHandle nodeHandle_;
    ros::Publisher XavPublisher_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber objectSubscriber_;
    ros::Subscriber XavSubscriber_;
	
    double CycleTime_ = 0.0;
    int index_;
    bool Alpha_ = false;

    //image
    LaneDetect::LaneDetector laneDetector_;
    bool viewImage_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_;
    bool Beta_ = false;

    float AngleDegree_; // -1 ~ 1  - Twist msg angular.z
    float TargetVel_ = 0.0f; // -1 ~ 1  - Twist msg linear.x
    float SafetyVel_;
    float ResultVel_;
    float FVmaxVel_;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_ = 0.8f;
    float distAngle_ = 0.0f;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    bool Gamma_ = false;

    //ZMQ
    ZMQ_CLASS ZMQ_SOCKET_;
    ZmqData* zmq_data_;

    //Thread
    std::thread controlThread_;
    std::thread laneDetectThread_;
    std::thread objectDetectThread_;
    std::thread tcpThread_;

    std::mutex image_mutex_;
    std::mutex object_mutex_;
    std::mutex lane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;

    std::condition_variable cv_;

    obstacle_detector::Obstacles Obstacle_;
    boost::shared_mutex mutexObjectCallback_;

    bool imageStatus_ = false;
    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_, camImageTmp_;
    bool droi_ready_ = false;

    bool isNodeRunning_ = true;
    bool controlDone_ = false;

    float CurVel_ = 0.0f;
    float RefVel_ = 0.0f;
     
    bool getImageStatus(void);	
    void* lanedetectInThread();
    void* objectdetectInThread();
    void reply(ZmqData* zmq_data);
    void displayConsole();
    void spin();
};

} /* namespace scale_truck_control */
