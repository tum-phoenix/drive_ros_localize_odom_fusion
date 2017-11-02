#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

// constructor (initialize everything)
ImuOdoOdometry::ImuOdoOdometry(ros::NodeHandle& pnh):
  pnh_(pnh)
{
  // queue for subscribers and sync policy
  int queue_size;

  // file path
  std::string debug_file_path;

  // ros parameters
  pnh.param<int>("queue_size", queue_size, 5);
  pnh.param<int>("queue_size", steps_to_predict_without_data, 5);
  pnh.param<std::string>("tf_parent", tf_parent, "odometry");
  pnh.param<std::string>("tf_child", tf_child, "rear_axis_middle_ground");
  pnh.param<std::string>("debug_file_path", debug_file_path, "~/debug.csv");
  pnh.param<bool>("debug_rviz", debug_rviz, false);
  pnh.param<bool>("debug_file", debug_file, false);

  // debug publisher
  if(debug_rviz)
  {
    vis_pub = pnh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  }

  // debug file
  if(debug_file)
  {
     file_log.open( debug_file_path );
     file_log << "timestamp,"
              << "delta,"
              << "state_x,"
              << "state_y,"
              << "state_theta,"
              << "state_v,"
              << "state_a,"
              << "state_omega,"
              << "meas_ax,"
              << "meas_ay,"
              << "meas_v,"
              << "meas_omega"
              << std::endl;
  }

  // init subscribers
  odo_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA>(pnh, "odo_in", queue_size);
  imu_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU>(pnh, "imu_in", queue_size);

  policy = new SyncPolicy(queue_size);

  sync = new message_filters::Synchronizer<SyncPolicy>(static_cast<SyncPolicy>(*policy), *odo_sub, *imu_sub);
  sync->registerCallback(boost::bind(&ImuOdoOdometry::syncCallback, this, _1, _2));

  // reset initial times
  odo_msg.header.stamp = ros::Time(0);
  imu_msg.header.stamp = ros::Time(0);

  // Init kalman
  State s;
  s.setZero();
  filter.init(s);

  // Set initial state covariance
  Kalman::Covariance<State> stateCov;
  stateCov.setZero();

  if( pnh_.getParam("kalman_cov/filter_init_var_x", stateCov(State::X, State::X)) &&
      pnh_.getParam("kalman_cov/filter_init_var_y", stateCov(State::Y, State::Y)) &&
      pnh_.getParam("kalman_cov/filter_init_var_a", stateCov(State::A, State::A)) &&
      pnh_.getParam("kalman_cov/filter_init_var_v", stateCov(State::V, State::V)) &&
      pnh_.getParam("kalman_cov/filter_init_var_theta", stateCov(State::THETA, State::THETA)) &&
      pnh_.getParam("kalman_cov/filter_init_var_omega", stateCov(State::OMEGA, State::OMEGA)))
  {
    ROS_INFO("Kalman initial state covariance loaded successfully");
  }else{
    ROS_ERROR("Error loading Kalman initial state covariance!");
    throw std::runtime_error("Error loading parameters");
  }

  filter.setCovariance(stateCov);

  // Set process noise covariance
  Kalman::Covariance<State> cov;
  cov.setZero();

  if( pnh_.getParam("kalman_cov/sys_var_x", cov(State::X, State::X)) &&
      pnh_.getParam("kalman_cov/sys_var_y", cov(State::Y, State::Y)) &&
      pnh_.getParam("kalman_cov/sys_var_a", cov(State::A, State::A)) &&
      pnh_.getParam("kalman_cov/sys_var_v", cov(State::V, State::V)) &&
      pnh_.getParam("kalman_cov/sys_var_theta", cov(State::THETA, State::THETA)) &&
      pnh_.getParam("kalman_cov/sys_var_omega", cov(State::OMEGA, State::OMEGA)))
  {
    ROS_INFO("Kalman initial process covariance loaded successfully");
  }else{
    ROS_ERROR("Error loading Kalman initial process covariance!");
    throw std::runtime_error("Error loading parameters");
  }

  sys.setCovariance(cov);

  // Set sensor covariances
  if( pnh_.getParam("sensor_cov/odometer_velo_cov_xx", odometer_velo_cov_xx) &&
      pnh_.getParam("sensor_cov/imu_acc_cov_xx", imu_acc_cov_xx) &&
      pnh_.getParam("sensor_cov/imu_acc_cov_yy", imu_acc_cov_yy) &&
      pnh_.getParam("sensor_cov/imu_gyro_cov_zz", imu_gyro_cov_zz) &&
      pnh_.getParam("sensor_cov/imu_acc_bias_x", imu_acc_bias_x) &&
      pnh_.getParam("sensor_cov/imu_acc_bias_y", imu_acc_bias_y) &&
      pnh_.getParam("sensor_cov/imu_acc_bias_z", imu_acc_bias_z) &&
      pnh_.getParam("sensor_cov/imu_gyro_bias_x", imu_gyro_bias_x) &&
      pnh_.getParam("sensor_cov/imu_gyro_bias_y", imu_gyro_bias_y) &&
      pnh_.getParam("sensor_cov/imu_gyro_bias_z", imu_gyro_bias_z))
  {
    ROS_INFO("Sensor covariances loaded successfully");
  }else{
    ROS_ERROR("Error loading sensor covariances!");
    throw std::runtime_error("Error loading parameters");
  }


  // initialize vars
  ct_no_data = 0;

}



ImuOdoOdometry::~ImuOdoOdometry()
{
  file_log.close();
}

void ImuOdoOdometry::syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                                  const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu)
{
  mut.lock();
  odo_msg = *msg_odo;
  imu_msg = *msg_imu;
  mut.unlock();

  ROS_DEBUG_STREAM("Got new callback with time: " << msg_imu->header.stamp);

}


void ImuOdoOdometry::computeOdometry()
{

  mut.lock();
  const drive_ros_msgs::mav_cc16_ODOMETER_DELTA local_odo = odo_msg;
  const drive_ros_msgs::mav_cc16_IMU local_imu = imu_msg;
  mut.unlock();


  // set timestamp
  currentTimestamp = local_imu.header.stamp; // both imu and odo should be the same time


  // check if timestamps are the same
  if(local_odo.header.stamp != local_imu.header.stamp)
  {
    ROS_WARN("Odometry and IMU timestamps are not the same!");
    return;
  }


  // check if timestamps are 0
  if(ros::Time(0) == currentTimestamp)
  {
    ROS_WARN("Didn't receive any sensory message yet. Waiting...");
    return;
  }

  // Compute Measurement Update
  computeMeasurement(local_odo, local_imu);

  // Compute Kalman filter step
  if(computeFilterStep())
  {
    // Update car state
    publishCarState();
  }

  // save timestamp
  lastTimestamp = currentTimestamp;

}

void ImuOdoOdometry::computeMeasurement(const drive_ros_msgs::mav_cc16_ODOMETER_DELTA &odo_msg,
                                        const drive_ros_msgs::mav_cc16_IMU &imu_msg)
{
  T v = 0;
  T ax = 0;
  T ay = 0;
  T omega = 0;

  T axVar = 0;
  T ayVar = 0;
  T vVar = 0;
  T omegaVar = 0;

  v = odo_msg.velocity.x;
  vVar = odometer_velo_cov_xx;

  // TODO: remove acceleration orientation hack
  omega = imu_msg.gyro.z;
  ax    = GRAVITY * imu_msg.acc.x;
  ay    = GRAVITY * imu_msg.acc.y;

  omegaVar = imu_gyro_cov_zz;
  axVar    = GRAVITY * GRAVITY * imu_acc_cov_xx;
  ayVar    = GRAVITY * GRAVITY * imu_acc_cov_yy;


  // Set actual measurement vector
  //TODO fail, if v gets smaller, the filter fails!
  z.v() = v;
  z.ax() = ax;
  z.ay() = ay;
  z.omega() = omega;


  // Set measurement covariances
  Kalman::Covariance< Measurement > cov;
  cov.setZero();
  cov(Measurement::AX,    Measurement::AX)    = axVar;
  cov(Measurement::AY,    Measurement::AY)    = ayVar;
  cov(Measurement::V,     Measurement::V)     = vVar;
  cov(Measurement::OMEGA, Measurement::OMEGA) = omegaVar;
  mm.setCovariance(cov);

  ROS_DEBUG_STREAM("measurementVector: " << z);
  ROS_DEBUG_STREAM("measurementCovariance:" << mm.getCovariance());

  //Error checking
  if(omega != omega){
      throw std::runtime_error("omega is NAN");
  }
  if(ay != ay){
      throw std::runtime_error("ay is NAN");
  }
  if(ax != ax){
      throw std::runtime_error("ax is NAN");
  }
  if(v != v){
      throw std::runtime_error("v is NAN");
  }
}

bool ImuOdoOdometry::computeFilterStep()
{

  // check if this is first loop
  if(ros::Time(0) == lastTimestamp){
    lastTimestamp = currentTimestamp;
  }


  // time since UKF was last called (parameter, masked as control input)
  currentDelta = ( currentTimestamp - lastTimestamp );
  if(currentDelta == ros::Duration(0)) {
      // Just predict using previous delta
      ROS_WARN_STREAM("No new sensor data."
          << " last = " << lastTimestamp
          << " current = " << currentTimestamp);

  } else if( currentDelta < ros::Duration(0)) {
      ROS_WARN_STREAM("Jumping backwards in time."
          << " last = " << lastTimestamp
          << " current = " << currentTimestamp
          << " delta = " << currentDelta);
      return false;
  }

  ROS_DEBUG_STREAM("[delta]" << "delta current: " << currentDelta << " delta previous: " << previousDelta);

  if(ros::Duration(0) == currentDelta && ct_no_data < steps_to_predict_without_data) {

      // increase no data counter
      ct_no_data++;

      // use time delta from previous step
      u.dt() = previousDelta.toSec();

      // Prediction only
      // predict state for current time-step using the kalman filter
      filter.predict(sys, u);
  } else if(ros::Duration(0) != currentDelta) {

      // new data available, reset counter
      ct_no_data = 0;

      // get current time delta
      u.dt() = currentDelta.toSec();

      // Prediction + update
      // predict state for current time-step using the kalman filter
      filter.predict(sys, u);
      // perform measurement update
      filter.update(mm, z);

      // Update previous delta
      previousDelta = currentDelta;
  } else {

    // we didn't get new data for a longer time
    return false;
  }

  ROS_DEBUG_STREAM("stateCovariance" << filter.getCovariance());

  return true;
}

void ImuOdoOdometry::publishCarState()
{
  const auto& state = filter.getState();
  ROS_DEBUG_STREAM("newState: " << state);

  // publish tf
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = currentTimestamp;
  transformStamped.header.frame_id = tf_parent;
  transformStamped.child_frame_id = tf_child;
  transformStamped.transform.translation.x = state.x();
  transformStamped.transform.translation.y = state.y();
  transformStamped.transform.translation.z = 0;
  tf2::Quaternion q1;
  q1.setRPY(0, 0, state.theta());
  transformStamped.transform.rotation.x =  q1.x();
  transformStamped.transform.rotation.y =  q1.y();
  transformStamped.transform.rotation.z =  q1.z();
  transformStamped.transform.rotation.w =  q1.w();
  br.sendTransform(transformStamped);


  // debug to file
  if(debug_file)
  {
     file_log << currentTimestamp         << ","
              << currentDelta             << ","
              << state.x()                << ","
              << state.y()                << ","
              << state.theta()            << ","
              << state.v()                << ","
              << state.a()                << ","
              << state.omega()            << ","
              << z.ax()                   << ","
              << z.ay()                   << ","
              << z.v()                    << ","
              << z.omega()
              << std::endl;
  }

  // debug to rviz
  if(debug_rviz)
  {
    ct++;
    visualization_msgs::Marker marker;
    marker.header.frame_id = tf_parent;
    marker.header.stamp = currentTimestamp;
    marker.ns = "odometry";
    marker.id = ct;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = state.x();
    marker.pose.position.y = state.y();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q1.x();
    marker.pose.orientation.y = q1.y();
    marker.pose.orientation.z = q1.z();
    marker.pose.orientation.w = q1.w();
    marker.scale.x = 0.05;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(30);
    vis_pub.publish(marker);
  }

}

