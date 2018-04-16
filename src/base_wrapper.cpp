#include "drive_ros_localize_odom_fusion/base_wrapper.h"


bool BaseWrapper::initROS(bool use_bag)
{
  // queue for subscribers and sync policy
  int queue_size;

  // file path
  std::string debug_out_file_path, odo_out_topic;

  // ros parameters
  pnh.param<int>("queue_size", queue_size, 5);
  pnh.param<std::string>("static_frame", static_frame, "odometry");
  pnh.param<std::string>("moving_frame", moving_frame, "rear_axis_middle_ground");
  pnh.param<bool>("use_sensor_time_for_pub", use_sensor_time_for_pub, false);

  pnh.param<std::string>("odo_topic_name", odo_topic_name, "/odo");
  pnh.param<std::string>("imu_topic_name", imu_topic_name, "/imu");
  pnh.param<std::string>("odo_out_topic", odo_out_topic, "/odom");

  pnh.param<std::string>("debug_out_file_path", debug_out_file_path, "/tmp/odom_debug.csv");
  pnh.param<bool>("debug_out", debug_out_file, false);

  float max_time_between_meas_fl;
  pnh.param<float>("max_time_between_meas", max_time_between_meas_fl, 0.5);
  max_time_between_meas = ros::Duration(max_time_between_meas_fl);

  // odometry publisher
  odo_pub = nh.advertise<nav_msgs::Odometry>(odo_out_topic, 0);

  // debug file
  if(debug_out_file){
    writeOutputHeader(debug_out_file_path);
  }

  // input of messages (read from bag file or create real subscribers)
  if(use_bag){

    // fake subscriber
    odo_sub = new message_filters::Subscriber<nav_msgs::Odometry>();
    imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>();


  }else{

    // real subscribers
    odo_sub = new message_filters::Subscriber<nav_msgs::Odometry>(pnh, odo_topic_name, queue_size);
    imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(pnh, imu_topic_name, queue_size);

  }

  // initialize policy and register sync callback
  policy = new SyncPolicy(queue_size);
  sync = new message_filters::Synchronizer<SyncPolicy>(static_cast<SyncPolicy>(*policy), *odo_sub, *imu_sub);
  sync->registerCallback(boost::bind(&BaseWrapper::syncCallback, this, _1, _2));

  // parameters can be found here: http://wiki.ros.org/message_filters/ApproximateTime
  double age_penalty, odo_topic_rate, imu_topic_rate, max_time_between_imu_odo;
  pnh.param<double>("age_penalty", age_penalty, 300);
  pnh.param<double>("odo_topic_rate", odo_topic_rate, 300);
  pnh.param<double>("imu_topic_rate", imu_topic_rate, 300);
  pnh.param<double>("max_time_between_imu_odo", max_time_between_imu_odo, 0.1);

  policy->setAgePenalty(age_penalty);
  policy->setMaxIntervalDuration(ros::Duration(max_time_between_imu_odo));

  // lower bound should be half of the time period (= double the rate) for each topic
  policy->setInterMessageLowerBound(0, ros::Rate(odo_topic_rate*2).expectedCycleTime());
  policy->setInterMessageLowerBound(1, ros::Rate(imu_topic_rate*2).expectedCycleTime());


  // init services
  reload_proc_cov = pnh.advertiseService("reload_proc_cov", &BaseWrapper::svrReloadProcCov, this);
  reinit_state =    pnh.advertiseService("reinit_state",    &BaseWrapper::svrReinitState,   this);

  // reset initial times
  last_timestamp        = ros::Time(0);
  last_delta            = ros::Duration(0);

  // initialize Kalman filter state & covariances
  return initFilterState() && initFilterProcessCov();

}


// callback if both odo and imu messages with same timestamp have arrived
void BaseWrapper::syncCallback(const nav_msgs::OdometryConstPtr &msg_odo,
                               const sensor_msgs::ImuConstPtr &msg_imu)
{

  ROS_DEBUG_STREAM("Got new callback with times. IMU: " << msg_imu->header.stamp
                                           << " Odom: " << msg_odo->header.stamp
                                           << " Diff: " << msg_imu->header.stamp.toSec() - msg_odo->header.stamp.toSec());

  // check if timestamps are 0
  if(ros::Time(0) == msg_imu->header.stamp || ros::Time(0) == msg_odo->header.stamp)
  {
    ROS_WARN("A timestamp is 0. Skipping messages.");
    return;
  }

  // set timestamp
  ros::Time current_timestamp =
      ros::Time((msg_imu->header.stamp.toSec() + msg_odo->header.stamp.toSec())/2);


  // check if this is first loop or reinitialized
  ros::Duration current_delta;
  if(ros::Time(0) == last_timestamp){
    ROS_WARN("Last timestamp is 0. Using max_time_between_meas/2 as delta.");
    last_timestamp = current_timestamp - ros::Duration(max_time_between_meas.toSec()/2); // use some value for first timestamp
    current_delta = ros::Duration(max_time_between_meas.toSec()/2);                      // use some value for first delta
  }else{
    current_delta = ( current_timestamp - last_timestamp );
  }


  // time jump to big -> reset filter
  if(current_delta > max_time_between_meas){

    ROS_ERROR_STREAM("Delta Time Threshold exceeded. Reinit Filter."
        << " delta = " << current_delta
        << " thres = " << max_time_between_meas
        << " lastTime = " << last_timestamp
        << " currTime = " << current_timestamp);

   // reset covariances
   initFilterProcessCov();
   last_timestamp = ros::Time(0);
   return;

  // jumping back in time
  }else if(current_delta < ros::Duration(0)) {
      ROS_WARN_STREAM("Jumping back in time. Delta = " << current_delta);

      // reset times
      current_timestamp = last_timestamp;
      current_delta = ros::Duration(0);

      // go on and hope that the rest of the data is ok

  // no new data avialable
  }else if(ros::Duration(0) == current_delta){

      // use last delta
      current_delta = last_delta;
      ROS_WARN_STREAM("Time delta is zero. Using old delta: " << last_delta);

  }

  // insert measurement update
  if(!insertMeasurement(msg_odo, msg_imu))
  {
    last_timestamp = ros::Time(0);
    return;
  }


  // compute Kalman filter step
  computeFilterStep(current_delta.toSec(), msg_odo, msg_imu);

  // create output messages
  geometry_msgs::TransformStamped tf;
  nav_msgs::Odometry odom;

  // set frames
  tf.header.frame_id = static_frame;
  tf.child_frame_id = moving_frame;
  odom.header.frame_id = static_frame;
  odom.child_frame_id = moving_frame;

  // set output time
  if(use_sensor_time_for_pub){
    tf.header.stamp = current_timestamp;
    odom.header.stamp = current_timestamp;
  }else{
    tf.header.stamp = ros::Time::now();
    odom.header.stamp = ros::Time::now();
  }

  // get output from wrapper
  getOutput(tf, odom);

  // publish
  odo_pub.publish(odom);
  br.sendTransform(tf);

  // debug to file
  if(debug_out_file)
  {
    writeOutputResult(&odom);
  }

  // save timestamp
  last_timestamp = current_timestamp;
  last_delta = current_delta;

}

// write header file of output log file
void BaseWrapper::writeOutputHeader(std::string filename)
{
  file_out_log.open( filename );

  file_out_log << "timestamp,";

  file_out_log << "pose_posX,"
               << "pose_posY,"
               << "pose_posZ,"
               << "pose_oriX,"
               << "pose_oriY,"
               << "pose_oriZ,";

  for(int i=0; i<36; i++)
    file_out_log << "pose_cov_[" << i << "],";

  file_out_log << "twist_linX,"
               << "twist_linY,"
               << "twist_linZ,"
               << "twist_angX,"
               << "twist_angY,"
               << "twist_angZ,";

  for(int i=0; i<36; i++)
    file_out_log << "twist_cov_[" << i << "],";

   file_out_log << std::endl;
}

// write the odometry message to output log file
void BaseWrapper::writeOutputResult(const nav_msgs::Odometry *msg)
{
  file_out_log << msg->header.stamp.toSec() << ",";

  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  file_out_log << msg->pose.pose.position.x << ","
               << msg->pose.pose.position.y << ","
               << msg->pose.pose.position.z << ","
               << roll  << ","
               << pitch << ","
               << yaw   << ",";

  for(int i=0; i<36; i++)
    file_out_log << msg->pose.covariance.at(i) << ",";

  file_out_log << msg->twist.twist.linear.x << ","
               << msg->twist.twist.linear.y << ","
               << msg->twist.twist.linear.z << ","
               << msg->twist.twist.angular.x << ","
               << msg->twist.twist.angular.y << ","
               << msg->twist.twist.angular.z << ",";

  for(int i=0; i<36; i++)
    file_out_log << msg->twist.covariance.at(i) << ",";

  file_out_log << std::endl;
}

// read data from bag and feed it into fake subscriber
bool BaseWrapper::processBag(std::string bag_file_path)
{
  // open bag
  rosbag::Bag bag;
  bag.open(bag_file_path, rosbag::bagmode::Read);

  // topics to load
  std::vector<std::string> bag_topics;
  bag_topics.push_back(odo_topic_name);
  bag_topics.push_back(imu_topic_name);

  // create bag view
  rosbag::View bag_view(bag, rosbag::TopicQuery(bag_topics));

  // loop over all messages
  BOOST_FOREACH(rosbag::MessageInstance const m, bag_view)
  {

    // odometer msg
    if(m.getTopic() == odo_topic_name || ("/" + m.getTopic() == odo_topic_name))
    {
      nav_msgs::OdometryConstPtr odo = m.instantiate<nav_msgs::Odometry>();
      if (odo != NULL){
        sync->add<0>(odo);
      }
    }

    // imu msg
    if(m.getTopic() == imu_topic_name || ("/" + m.getTopic() == imu_topic_name))
    {
      sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      if (imu != NULL){
        sync->add<1>(imu);
      }
    }
  }

  // close bag
  bag.close();
  return true;
}

// reload process covariances
bool BaseWrapper::svrReloadProcCov(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res)
{
  initFilterProcessCov();

  res.message = "Kalman filter process covariances reloaded from parameter server.";
  return res.success = true;
}

// reinit kalman state
bool BaseWrapper::svrReinitState(std_srvs::Trigger::Request  &req,
                                    std_srvs::Trigger::Response &res)
{
  initFilterState();
  last_timestamp = ros::Time(0);

  res.message = "Kalman filter state reinitialized. Set state to 0 and load initial state covariances";
  return res.success = true;
}


