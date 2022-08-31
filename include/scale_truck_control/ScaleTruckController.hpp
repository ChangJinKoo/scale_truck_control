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
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <obstacle_detector/Obstacles.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include "lane_detect/lane_detect.hpp"
#include "zmq_class/zmq_class.h"

//custom msgs
#include <scale_truck_control/lrc2xav.h>
#include <scale_truck_control/xav2lrc.h>
#include <scale_truck_control/yolo_flag.h>
#include <yolo_object_detection/bounding_box.h>

namespace scale_truck_control {

class ScaleTruckController {
  public:
    explicit ScaleTruckController(ros::NodeHandle nh);

    ~ScaleTruckController();

  private:
    bool readParameters();

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void rearImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void objectCallback(const obstacle_detector::Obstacles &msg);
    void XavSubCallback(const scale_truck_control::lrc2xav &msg);
    void ScanErrorCallback(const std_msgs::UInt32::ConstPtr &msg);
    void bboxCallback(const yolo_object_detection::bounding_box &msg);
    void recordData(struct timeval startTime);
    void imageCompress(cv::Mat camImage);
    void reply(ZmqData* zmq_data);
    void requestImage(ImgData* img_data);
    void replyImage(); 
    void displayConsole();
    void spin();
    bool getImageStatus(void);	
    
    ros::NodeHandle nodeHandle_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber rearImageSubscriber_;
    ros::Subscriber objectSubscriber_;
    ros::Subscriber XavSubscriber_;
    ros::Subscriber ScanSubError;	
    ros::Subscriber bboxSubscriber_;	
    ros::Publisher XavPublisher_;
    ros::Publisher runYoloPublisher_;

    double CycleTime_ = 0.0;
    int index_;
    float RCMVel_;
    float RCMDist_;
    bool fi_encoder_ = false;
    bool alpha_ = false;
    bool send_rear_camera_image_ = false;
    uint8_t lrc_mode_ = 0;
    uint8_t crc_mode_ = 0;

    //image
    LaneDetect::LaneDetector laneDetector_;
    bool viewImage_;
    bool rear_camera_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_;
    bool fi_camera_ = false;
    bool beta_ = false;

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
    float ampersand_ = 0.0f;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    uint32_t LdrErrMsg_;
    bool fi_lidar_ = false;
    bool gamma_ = false;

    //bbox
    uint32_t x_ = 0;
    uint32_t y_ = 0;
    uint32_t w_ = 0;
    uint32_t h_ = 0;
    
    //ZMQ
    ZMQ_CLASS ZMQ_SOCKET_;
    ZmqData* zmq_data_;
    ImgData* img_data_;

    //Thread
    std::thread controlThread_;
    std::thread laneDetectThread_;
    std::thread objectDetectThread_;
    std::thread tcpThread_;
    std::thread tcpImgReqThread_;
    std::thread tcpImgRepThread_;

    std::mutex image_mutex_;
    std::mutex rear_image_mutex_;
    std::mutex object_mutex_;
    std::mutex lane_mutex_;
    std::mutex vel_mutex_;
    std::mutex dist_mutex_;
    std::mutex rep_mutex_;
    std::mutex mode_mutex_;
    std::mutex bbox_mutex_;

    std::condition_variable cv_;

    obstacle_detector::Obstacles Obstacle_;
    boost::shared_mutex mutexObjectCallback_;

    bool imageStatus_ = false;
    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_, camImageTmp_;
    cv::Mat rearImageCopy_, rearImageTmp_, rearImageJPEG_;
    bool droi_ready_ = false;

    bool isNodeRunning_ = true;
    bool controlDone_ = false;

    float CurVel_ = 0.0f;
    float RefVel_ = 0.0f;
     
    void* lanedetectInThread();
    void* objectdetectInThread();

    bool run_yolo_ = false;
    bool tcp_img_req_ = false;
    bool tcp_img_rep_ = false;
    int req_check_ = 0;
    int rep_check_ = 0;
    double time_ = 0.0;
    double DelayTime_ = 0.0;
    std::vector<uchar> compImageSend_;
    std::vector<uchar> compImageRecv_;
};

} /* namespace scale_truck_control */
