#include "drive_ros_localize_odom_fusion/base_wrapper.h"
#include "drive_ros_localize_odom_fusion/save_odom_in_CSV.h"

bool BaseWrapper::initROS()
{
  /*
   * #######################
   * Parameter and Publisher
   * #######################
   */

  // queue for subscribers and sync policy
  int queue_size;

  // file path
  std::string debug_out_file_path, odo_out_topic;

  // ros parameters
  pnh.param<std::string>("static_frame", static_frame, "");
  pnh.param<std::string>("moving_frame", moving_frame, "");

  pnh.param<std::string>("pred_odo_topic_name", pred_odo_topic, "");
  pnh.param<std::string>("pred_imu_topic_name", pred_imu_topic, "");
  pnh.param<std::string>("corr_odo_topic_name", corr_odo_topic, "");
  pnh.param<std::string>("corr_imu_topic_name", corr_imu_topic, "");

  pnh.param<std::string>("odo_out_topic", odo_out_topic, "/odom");

  pnh.param<int>("queue_size", queue_size, 5);
  pnh.param<std::string>("debug_out_file_path", debug_out_file_path, "/tmp/odom_debug.csv");
  pnh.param<bool>("debug_out", debug_out_file, false);

  float time_threshold_fl;
  pnh.param<float>("time_threshold", time_threshold_fl, 0.5);
  time_threshold = ros::Duration(time_threshold_fl);

  // odometry publisher
  odo_pub = nh.advertise<nav_msgs::Odometry>(odo_out_topic, 0);

  // debug file
  if(debug_out_file){
    ROS_INFO_STREAM("Debug to file: " << debug_out_file_path);
    SaveOdomInCSV::writeHeader(debug_out_file_path, file_out_log);
  }

  // init services
  reload_proc_cov = pnh.advertiseService("reset", &BaseWrapper::svrReset, this);

  /* ###########################
   * PREDICTION subscriber setup
   * ###########################
   */

  // only IMU data is available for prediction
  if(pred_odo_topic.empty()){

    ROS_INFO_STREAM("Setup single prediction subscriber for: " << pred_imu_topic);
    pred_imu_single_sub = pnh.subscribe(pred_imu_topic, queue_size, &BaseWrapper::predImuCallback, this);

  // only odometry data is available for prediction
  }else if(pred_imu_topic.empty()){

    ROS_INFO_STREAM("Setup single prediction subscriber for: " << pred_odo_topic);
    pred_odo_single_sub = pnh.subscribe(pred_odo_topic, queue_size, &BaseWrapper::predOdoCallback, this);

  // both odometry and IMU data are available for prediction
  }else{

    ROS_INFO_STREAM("Setup synchronized prediction subscriber for: " << pred_odo_topic << " and " << pred_imu_topic);
    pred_odo_sub = new message_filters::Subscriber<nav_msgs::Odometry>(pnh, pred_odo_topic, queue_size);
    pred_imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(pnh, pred_imu_topic, queue_size);

    // initialize policy and register sync callback
    pred_policy = new SyncPolicy(queue_size);
    pred_sync = new message_filters::Synchronizer<SyncPolicy>(static_cast<SyncPolicy>(*pred_policy), *pred_odo_sub, *pred_imu_sub);
    pred_sync->registerCallback(boost::bind(&BaseWrapper::predSyncCallback, this, _1, _2));

    // parameters can be found here: http://wiki.ros.org/message_filters/ApproximateTime
    double age_penalty, odo_topic_rate, imu_topic_rate, max_time_between_imu_odo;
    pnh.param<double>("pred_age_penalty", age_penalty, 5);
    pnh.param<double>("pred_max_time_between_imu_odo", max_time_between_imu_odo, 0.01);
    pnh.param<double>("pred_odo_topic_rate", odo_topic_rate, 100);
    pnh.param<double>("pred_imu_topic_rate", imu_topic_rate, 100);

    pred_policy->setAgePenalty(age_penalty);
    pred_policy->setMaxIntervalDuration(ros::Duration(max_time_between_imu_odo));

    // lower bound should be half of the time period (= double the rate) for each topic
    pred_policy->setInterMessageLowerBound(0, ros::Rate(odo_topic_rate*2).expectedCycleTime());
    pred_policy->setInterMessageLowerBound(1, ros::Rate(imu_topic_rate*2).expectedCycleTime());
  }

  /* ###########################
   * CORRECTION subscriber setup
   * ###########################
   */

  // only IMU data is available for correction
  if(corr_odo_topic.empty()){

    ROS_INFO_STREAM("Setup single correction subscriber for: " << corr_imu_topic);
    corr_imu_single_sub = pnh.subscribe(corr_imu_topic, queue_size, &BaseWrapper::corrImuCallback, this);

  // only odometry data is available for correction
  }else if(corr_imu_topic.empty()){

    ROS_INFO_STREAM("Setup single correction subscriber for: " << corr_odo_topic);
    corr_odo_single_sub = pnh.subscribe(corr_odo_topic, queue_size, &BaseWrapper::corrOdoCallback, this);

  // both odometry and IMU data are available for correction
  }else{

    ROS_INFO_STREAM("Setup synchronized correction subscriber for: " << corr_odo_topic << " and " << corr_imu_topic);
    corr_odo_sub = new message_filters::Subscriber<nav_msgs::Odometry>(pnh, corr_odo_topic, queue_size);
    corr_imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(pnh, corr_imu_topic, queue_size);

    // initialize policy and register sync callback
    corr_policy = new SyncPolicy(queue_size);
    corr_sync = new message_filters::Synchronizer<SyncPolicy>(static_cast<SyncPolicy>(*corr_policy), *corr_odo_sub, *corr_imu_sub);
    corr_sync->registerCallback(boost::bind(&BaseWrapper::corrSyncCallback, this, _1, _2));

    // parameters can be found here: http://wiki.ros.org/message_filters/ApproximateTime
    double age_penalty, odo_topic_rate, imu_topic_rate, max_time_between_imu_odo;
    pnh.param<double>("corr_age_penalty", age_penalty, 5);
    pnh.param<double>("corr_max_time_between_imu_odo", max_time_between_imu_odo, 0.1);
    pnh.param<double>("corr_odo_topic_rate", odo_topic_rate, 100);
    pnh.param<double>("corr_imu_topic_rate", imu_topic_rate, 100);

    corr_policy->setAgePenalty(age_penalty);
    corr_policy->setMaxIntervalDuration(ros::Duration(max_time_between_imu_odo));

    // lower bound should be half of the time period (= double the rate) for each topic
    corr_policy->setInterMessageLowerBound(0, ros::Rate(odo_topic_rate*2).expectedCycleTime());
    corr_policy->setInterMessageLowerBound(1, ros::Rate(imu_topic_rate*2).expectedCycleTime());
  }

  // reset filter
  return reset();

}

bool BaseWrapper::reset()
{
  ROS_INFO("Reset Kalman Filter");

  // reset times
  pred_last_timestamp = ros::Time(0);
  pred_last_delta     = ros::Duration(0);
  corr_last_timestamp = ros::Time(0);
  corr_last_delta     = ros::Duration(0);

  // reset covariances and filter state
  model_mutex.lock();
  bool ret = initFilterState();
  model_mutex.unlock();
  return ret;
}

// reload process covariances
bool BaseWrapper::svrReset(std_srvs::Trigger::Request  &req,
                           std_srvs::Trigger::Response &res)
{
  res.message = "Reset Kalman Filter.";
  return res.success = reset();
}

// process timestamp and deltas
bool BaseWrapper::processTimestamp(ros::Time& last_t, ros::Time& curr_t,
                                   ros::Duration& last_d, ros::Duration& curr_d) const
{
  // check if current time is ok
  if(ros::Time(0) == curr_t)
  {
    ROS_ERROR("Current timestamp is 0. Aborting.");
    return false;
  }

  // check if this is first loop or reinitialized
  if(ros::Time(0) == last_t){
    ROS_INFO("Last timestamp is 0. Using time_threshold/5 as delta.");
    last_t = curr_t - ros::Duration(time_threshold.toSec()/5); // use some value for first timestamp
    curr_d = ros::Duration(time_threshold.toSec()/5);          // use some value for first delta
  }else{
    curr_d = ( curr_t - last_t );
  }

  // time jump to big -> reset filter
  if(curr_d > time_threshold){

    ROS_ERROR_STREAM("Delta Time Threshold exceeded. Reinit Filter."
        << " delta = " << curr_d
        << " thres = " << time_threshold
        << " lastTime = " << last_t
        << " currTime = " << curr_t);

   return false;

  // jumping back in time
  }else if(curr_d < ros::Duration(0)) {
      ROS_WARN_STREAM("Jumping back in time. Delta = " << curr_d <<
                       " old_time = " << last_t <<
                       " cur_time = " << curr_t);

      // reset times
      curr_t = last_t;
      curr_d = last_d;

  // no new data avialable
  }else if(ros::Duration(0) == curr_d){

      // use last delta
      curr_d = last_d;
      ROS_WARN_STREAM("Time delta is zero. Using old delta: " << last_d);

  // everything is ok -> save delta and time
  }else{
    last_d = curr_d;
    last_t = curr_t;
  }

  return true;
}

void BaseWrapper::predOdoCallback(const nav_msgs::OdometryConstPtr &msg_odo)
{
  // predict and output messages
  processPredictionData(msg_odo->header.stamp, msg_odo, NULL);
}

void BaseWrapper::predImuCallback(const sensor_msgs::ImuConstPtr &msg_imu)
{
  // predict and output messages
  processPredictionData(msg_imu->header.stamp, NULL, msg_imu);
}

void BaseWrapper::predSyncCallback(const nav_msgs::OdometryConstPtr &msg_odo,
                                   const sensor_msgs::ImuConstPtr &msg_imu)
{
  // predict and output messages
  processPredictionData(ros::Time((msg_imu->header.stamp.toSec() + msg_odo->header.stamp.toSec())/2.0),
                        msg_odo, msg_imu);
}

bool BaseWrapper::processPredictionData(ros::Time current_timestamp,
                                        const nav_msgs::OdometryConstPtr &msg_odo,
                                        const sensor_msgs::ImuConstPtr &msg_imu)
{
  // current delta
  ros::Duration current_delta;

  // process timestamp
  if(!processTimestamp(pred_last_timestamp, current_timestamp,
                       pred_last_delta, current_delta))
  {
    ROS_ERROR("Process prediction timestamp failed!");
    reset();
    return false;
  }

  // create output messages
  geometry_msgs::TransformStamped tf;
  nav_msgs::Odometry odom;

  // do the prediction
  model_mutex.lock();
  if(!predict(current_delta.toSec(), msg_odo, msg_imu))
  {
    ROS_ERROR("Prediction step failed!");
    model_mutex.unlock();
    reset();
    return false;
  }

  // get output from wrapper
  getOutput(tf, odom);
  model_mutex.unlock();

  // set frames
  tf.header.frame_id = static_frame;
  tf.child_frame_id =  moving_frame;
  odom.header.frame_id = static_frame;
  odom.child_frame_id =  moving_frame;

  // set output time
  tf.header.stamp = current_timestamp;
  odom.header.stamp = current_timestamp;

  // publish
  odo_pub.publish(odom);
  br.sendTransform(tf);

  // debug to file
  if(debug_out_file)
  {
    SaveOdomInCSV::writeMsg(odom, file_out_log);
  }

  return true;
}


void BaseWrapper::corrOdoCallback(const nav_msgs::OdometryConstPtr &msg_odo)
{
  // correct
  processCorrectionData(msg_odo->header.stamp, msg_odo, NULL);
}

void BaseWrapper::corrImuCallback(const sensor_msgs::ImuConstPtr &msg_imu)
{
  // correct
  processCorrectionData(msg_imu->header.stamp, NULL, msg_imu);
}

void BaseWrapper::corrSyncCallback(const nav_msgs::OdometryConstPtr &msg_odo,
                                   const sensor_msgs::ImuConstPtr &msg_imu)
{
  // correct
  processCorrectionData(ros::Time((msg_imu->header.stamp.toSec() + msg_odo->header.stamp.toSec())/2.0),
                        msg_odo, msg_imu);
}

bool BaseWrapper::processCorrectionData(ros::Time current_timestamp,
                                        const nav_msgs::OdometryConstPtr &msg_odo,
                                        const sensor_msgs::ImuConstPtr &msg_imu)
{
  // current delta
  ros::Duration current_delta;

  // process timestamp
  if(!processTimestamp(corr_last_timestamp, current_timestamp,
                       corr_last_delta, current_delta))
  {
    ROS_ERROR("Process correction timestamp failed!");
    reset();
    return false;
  }

  // do the correction
  model_mutex.lock();
  if(!correct(current_delta.toSec(), msg_odo, msg_imu))
  {
    ROS_ERROR("Correction step failed!");
    model_mutex.unlock();
    reset();
    return false;
  }

  model_mutex.unlock();
  return true;
}



