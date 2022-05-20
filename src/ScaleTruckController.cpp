#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), ZMQ_SOCKET_(nh){
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

ScaleTruckController::~ScaleTruckController() {
  isNodeRunning_ = false;

  scale_truck_control::xav2lrc msg;
  msg.tar_vel = ResultVel_;
  {
    std::scoped_lock lock(dist_mutex_);
    msg.steer_angle = AngleDegree_;
    msg.cur_dist = distance_;
  }
  {
    std::scoped_lock lock(rep_mutex_);
    msg.tar_dist = TargetDist_;
    msg.fi_encoder = fi_encoder_;
    msg.fi_camera = fi_camera_;
    msg.fi_lidar = fi_lidar_;
    msg.beta = beta_;
    msg.gamma = gamma_;
  }

  XavPublisher_.publish(msg);
  controlThread_.join();

  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  /***************/
  /* View Option */
  /***************/
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, true);

  /*******************/
  /* Velocity Option */
  /*******************/
  nodeHandle_.param("params/index", index_, 0);
  nodeHandle_.param("params/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("params/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("params/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  nodeHandle_.param("params/ref_vel", RefVel_, 0.0f); // m/s
  nodeHandle_.param("params/rcm_vel", RCMVel_, 0.8f);
  
  /*******************/
  /* Distance Option */
  /*******************/
  nodeHandle_.param("params/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("params/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("params/target_dist", TargetDist_, 0.8f); // m
  nodeHandle_.param("params/rcm_dist", RCMDist_, 0.8f);

  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");  
  
  gettimeofday(&laneDetector_.start_, NULL);
  
  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize; 
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
  nodeHandle_.param("subscribers/lrc_to_xavier/topic", XavSubTopicName, std::string("/lrc2xav_msg"));
  nodeHandle_.param("subscribers/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  nodeHandle_.param("publishers/xavier_to_lrc/topic", XavPubTopicName, std::string("/xav2lrc_msg"));
  nodeHandle_.param("publishers/xavier_to_lrc/queue_size", XavPubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  XavSubscriber_ = nodeHandle_.subscribe(XavSubTopicName, XavSubQueueSize, &ScaleTruckController::XavSubCallback, this);
  ScanSubError = nodeHandle_.subscribe("/scan_error", 1000, &ScaleTruckController::ScanErrorCallback, this);  

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control::xav2lrc>(XavPubTopicName, XavPubQueueSize);

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;

  /************/
  /* ZMQ Data */
  /************/
  zmq_data_ = new ZmqData;
  zmq_data_->src_index = index_;
  zmq_data_->tar_index = 20;  //Control center

  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  tcpThread_ = std::thread(&ScaleTruckController::reply, this, zmq_data_);
}

bool ScaleTruckController::getImageStatus(void){
  std::scoped_lock lock(image_mutex_);
  return imageStatus_;
}

void* ScaleTruckController::lanedetectInThread() {
  static int cnt = 10;
  Mat dst;
  std::vector<Mat>channels;
  int count = 0;
  float AngleDegree;
  {
    std::scoped_lock lock(rep_mutex_, image_mutex_);
    if((!camImageTmp_.empty()) && (cnt != 0) && (TargetVel_ > 0.001f))
    {
      bitwise_xor(camImageCopy_,camImageTmp_, dst);
      split(dst, channels);
      for(int ch = 0; ch<dst.channels();ch++) {
        count += countNonZero(channels[ch]);
      }
      {
        if(count == 0 && fi_camera_)
          cnt -= 1;
        else 
          cnt = 10;
      }
    }
    camImageTmp_ = camImageCopy_.clone();
  }
  {
    std::scoped_lock lock(vel_mutex_);
    laneDetector_.get_steer_coef(CurVel_);
  }
  {
    std::unique_lock<std::mutex> lock(lane_mutex_);
    cv_.wait(lock, [this] {return droi_ready_; });

    AngleDegree = laneDetector_.display_img(camImageTmp_, waitKeyDelay_, viewImage_);
    droi_ready_ = false;
  }
  if(cnt == 0){
    {
      std::scoped_lock lock(rep_mutex_);
      beta_ = true;
    }
    {
      std::scoped_lock lock(dist_mutex_);
      AngleDegree_ = -distAngle_;
    }
  }
  else{
    std::scoped_lock lock(dist_mutex_);
    AngleDegree_ = AngleDegree;
  }
}

void* ScaleTruckController::objectdetectInThread() {
  float dist, angle; 
  float dist_tmp, angle_tmp;
  dist_tmp = 10.f; 
  /**************/
  /* Lidar Data */
  /**************/
  {
    std::scoped_lock lock(object_mutex_);
    ObjSegments_ = Obstacle_.segments.size();
    ObjCircles_ = Obstacle_.circles.size();
  
    for(int i = 0; i < ObjCircles_; i++)
    {
      //dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
      dist = -Obstacle_.circles[i].center.x - Obstacle_.circles[i].true_radius;
      angle = atanf(Obstacle_.circles[i].center.y/Obstacle_.circles[i].center.x)*(180.0f/M_PI);
      if(dist_tmp >= dist) {
        dist_tmp = dist;
        angle_tmp = angle;
      }
    }
  }
  if(ObjCircles_ != 0)
  {
    std::scoped_lock lock(dist_mutex_);
    distance_ = dist_tmp;
    distAngle_ = angle_tmp;
  }
  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  {
    std::scoped_lock lock(lane_mutex_);
    if(dist_tmp < 1.24 && dist_tmp > 0.30) // 1.26 ~ 0.28
    {
      laneDetector_.distance_ = (int)((1.24 - dist_tmp)*490.0);
    }
    else {
      laneDetector_.distance_ = 0;
    }
    droi_ready_ = true;
    cv_.notify_one();
  }  
  if(index_ == 0){  //LV
    std::scoped_lock lock(rep_mutex_, dist_mutex_);
    if(distance_ <= LVstopDist_) {
    // Emergency Brake
      ResultVel_ = 0.0f;
    }
    else if (distance_ <= SafetyDist_){
      float TmpVel_ = (ResultVel_-SafetyVel_)*((distance_-LVstopDist_)/(SafetyDist_-LVstopDist_))+SafetyVel_;
      if (TargetVel_ < TmpVel_){
        ResultVel_ = TargetVel_;
      }
      else{
        ResultVel_ = TmpVel_;
      }
    }
    else{
      ResultVel_ = TargetVel_;
    }
  }
  else{  //FVs
    std::scoped_lock lock(rep_mutex_, dist_mutex_);
    if ((distance_ <= FVstopDist_) || (TargetVel_ <= 0.1f)){
    // Emergency Brake
      ResultVel_ = 0.0f;
    }
    else {
      ResultVel_ = TargetVel_;
    }
  }
}

void ScaleTruckController::reply(ZmqData* zmq_data){
  while(isNodeRunning_){
    static float t_vel = 0.0f;
    static float t_dist = 0.8f;
    if(zmq_data->tar_index == 20){
      {
        std::scoped_lock lock(vel_mutex_, dist_mutex_);
	zmq_data->cur_vel = CurVel_;
	zmq_data->cur_dist = distance_;
	zmq_data->cur_angle = AngleDegree_;
      }
      {
        std::scoped_lock lock(lane_mutex_);
        zmq_data->coef[0].a = laneDetector_.lane_coef_.left.a;
        zmq_data->coef[0].b = laneDetector_.lane_coef_.left.b;
        zmq_data->coef[0].c = laneDetector_.lane_coef_.left.c;
        zmq_data->coef[1].a = laneDetector_.lane_coef_.right.a;
        zmq_data->coef[1].b = laneDetector_.lane_coef_.right.b;
        zmq_data->coef[1].c = laneDetector_.lane_coef_.right.c;
        zmq_data->coef[2].a = laneDetector_.lane_coef_.center.a;
        zmq_data->coef[2].b = laneDetector_.lane_coef_.center.b;
        zmq_data->coef[2].c = laneDetector_.lane_coef_.center.c;
      }
      ZMQ_SOCKET_.replyZMQ(zmq_data);
    }
    {
      std::scoped_lock lock(rep_mutex_, mode_mutex_);
      if(index_ == 0){
        if(crc_mode_ == 2){
          TargetVel_ = 0;
	  TargetDist_ = 0;
	}
	else if(crc_mode_ == 1){
          if(t_vel > RCMVel_) TargetVel_ = RCMVel_;
          else TargetVel_ = t_vel;
	  if(t_dist < RCMDist_) TargetDist_ = RCMDist_;
	  else TargetDist_ = t_dist;
	}
      }
      if(ZMQ_SOCKET_.rep_recv_->src_index == 20){
        if(index_ == 0){  //LV 
          t_vel = ZMQ_SOCKET_.rep_recv_->tar_vel;
          t_dist = ZMQ_SOCKET_.rep_recv_->tar_dist;
	  if(crc_mode_ == 1){  //RCM
            if (t_vel > RCMVel_) TargetVel_ = RCMVel_;
	    else TargetVel_ = t_vel;
	    if (t_dist < RCMDist_) TargetDist_ = RCMDist_;
	    else TargetDist_ = t_dist;
	  }
	  else{  //TM
            TargetVel_ = t_vel;
            TargetDist_ = t_dist;
	  }
	}
	fi_encoder_ = ZMQ_SOCKET_.rep_recv_->fi_encoder;
        fi_camera_ = ZMQ_SOCKET_.rep_recv_->fi_camera;
        fi_lidar_ = ZMQ_SOCKET_.rep_recv_->fi_lidar;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
void ScaleTruckController::displayConsole() {
  static std::string ipAddr = ZMQ_SOCKET_.getIPAddress();

  printf("\033[2J");
  printf("\033[1;1H");
  printf("%s (%s) - %s\n","-Client", ipAddr.c_str() , ZMQ_SOCKET_.udp_ip_.c_str());
  printf("\nAngle\t\t\t: %2.3f degree", AngleDegree_);
  printf("\nRefer Vel\t\t: %3.3f m/s", RefVel_);
  printf("\nSend Vel\t\t: %3.3f m/s", ResultVel_);
  printf("\nTar/Cur Vel\t\t: %3.3f / %3.3f m/s", TargetVel_, CurVel_);
  printf("\nTar/Cur Dist\t\t: %3.3f / %3.3f m", TargetDist_, distance_);
  printf("\nEncoder, Camera, Lidar Failure: %d / %d / %d", fi_encoder_, fi_camera_, fi_lidar_);
  printf("\nAlpha, Beta, Gamma\t: %d / %d / %d", alpha_, beta_, gamma_);
  printf("\nCRC mode, LRC mode\t: %d / %d", crc_mode_, lrc_mode_);
  printf("\nK1/K2\t\t\t: %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
  printf("\nLdrErrMsg\t\t\t: %x", LdrErrMsg_);
  if(ObjCircles_ > 0) {
    printf("\nCirs\t\t\t: %d", ObjCircles_);
    printf("\nDistAng\t\t\t: %2.3f degree", distAngle_);
  }
  if(ObjSegments_ > 0) {
    printf("\nSegs\t\t\t: %d", ObjSegments_);
  }
  printf("\nCycle Time\t\t: %3.3f ms", CycleTime_);
  printf("\n");
}

void ScaleTruckController::spin() {
  double diff_time=0.0;
  int cnt = 0;
  
  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!getImageStatus()) {
    printf("Waiting for image.\n");
    if(!isNodeRunning_) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }
  
  scale_truck_control::xav2lrc msg;
  scale_truck_control::lane_coef lane;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_ && ros::ok()) {
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();    

    if(enableConsoleOutput_)
      displayConsole();

    msg.tar_vel = ResultVel_;  //Xavier to LRC and LRC to OpenCR
    {
      std::scoped_lock lock(dist_mutex_);
      msg.steer_angle = AngleDegree_;
      msg.cur_dist = distance_;
    }
    {
      std::scoped_lock lock(rep_mutex_);
      msg.tar_dist = TargetDist_;
      msg.fi_encoder = fi_encoder_;
      msg.fi_camera = fi_camera_;
      msg.fi_lidar = fi_lidar_;
      msg.beta = beta_;
      msg.gamma = gamma_;
    }

    lane = laneDetector_.lane_coef_;
    XavPublisher_.publish(msg);

    if(!isNodeRunning_) {
      controlDone_ = true;
      ZMQ_SOCKET_.controlDone_ = true;
      ros::requestShutdown();
    }

    gettimeofday(&end_time, NULL);
    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
    cnt++;

    CycleTime_ = diff_time / (double)cnt;

    printf("cnt: %d\n", cnt);
    if (cnt > 3000){
      diff_time = 0.0;
      cnt = 0;
    }
  }
}

void ScaleTruckController::ScanErrorCallback(const std_msgs::UInt32::ConstPtr &msg) {
   LdrErrMsg_ = msg->data;
   if(fi_lidar_) {
     LdrErrMsg_ = 0x80008002;
   }
   {
     std::scoped_lock lock(rep_mutex_);
     if(LdrErrMsg_){
       gamma_ = true;
     }
   }
}

void ScaleTruckController::objectCallback(const obstacle_detector::Obstacles& msg) {
  {
    std::scoped_lock lock(object_mutex_);
    Obstacle_ = msg;
  }
}

void ScaleTruckController::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception : %s", e.what());
  }

  {
    std::scoped_lock lock(rep_mutex_, image_mutex_);
    if(cam_image && !fi_camera_) {
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
      imageStatus_ = true; 
    }
  }
}

void ScaleTruckController::XavSubCallback(const scale_truck_control::lrc2xav &msg){
  {
    std::scoped_lock lock(mode_mutex_);
    alpha_ = msg.alpha;
    lrc_mode_ = msg.lrc_mode;
    crc_mode_ = msg.crc_mode;
  }
  {
    std::scoped_lock lock(vel_mutex_);
    CurVel_ = msg.cur_vel;
  }
  if (index_ != 0){  //FVs
    std::scoped_lock lock(rep_mutex_);
    TargetVel_ = msg.tar_vel;
    TargetDist_ = msg.tar_dist;
  }
}

} /* namespace scale_truck_control */ 
