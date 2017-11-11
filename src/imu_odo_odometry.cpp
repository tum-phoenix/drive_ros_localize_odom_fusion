#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

// constructor (initialize everything)
ImuOdoOdometry::ImuOdoOdometry(ros::NodeHandle& pnh, ros::Rate& r):
  pnh_(pnh), rate(r)
{
  // queue for subscribers and sync policy
  int queue_size;

  // file path
  std::string debug_file_path;

  // ros parameters
  pnh_.param<int>("queue_size", queue_size, 5);
  pnh_.param<std::string>("tf_parent", tf_parent, "odometry");
  pnh_.param<std::string>("tf_child", tf_child, "rear_axis_middle_ground");
  pnh_.param<std::string>("debug_file_path", debug_file_path, "~/debug.csv");
  pnh_.param<bool>("debug_rviz", debug_rviz, false);
  pnh_.param<bool>("debug_file", debug_file, false);

  float reset_filter_thres_fl;
  pnh_.param<float>("reset_filter_thres", reset_filter_thres_fl, 0.5);
  reset_filter_thres = ros::Duration(reset_filter_thres_fl);

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

  // initialize Kalman filter state & covariances
  initFilterState();
  initFilterCov();

}


ImuOdoOdometry::~ImuOdoOdometry()
{
  file_log.close();
}

// initialize Filter State
void ImuOdoOdometry::initFilterState()
{
  ROS_INFO("Reset Kalman State");

  // Init kalman
  State s;
  s.setZero();
  filter.init(s);

  // reset no data counter
  ct_no_data = 0;
}

// initialize Filter covariances
void ImuOdoOdometry::initFilterCov()
{
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

  // reset initial times
  odo_msg.header.stamp = ros::Time(0);
  imu_msg.header.stamp = ros::Time(0);
  lastTimestamp = ros::Time(0);

}

// callback if both odo and imu messages with same timestamp have arrived
void ImuOdoOdometry::syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                                  const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu)
{
  mut.lock();
  odo_msg = *msg_odo;
  imu_msg = *msg_imu;
  mut.unlock();

  ROS_DEBUG_STREAM("Got new callback with time: " << msg_imu->header.stamp);

}


// this function is being called in a separate thread at a constant rate
void ImuOdoOdometry::computeOdometry()
{

  mut.lock();
  const drive_ros_msgs::mav_cc16_ODOMETER_DELTA local_odo = odo_msg;
  const drive_ros_msgs::mav_cc16_IMU local_imu = imu_msg;
  mut.unlock();


  // check if timestamps are the same
  if(local_odo.header.stamp != local_imu.header.stamp)
  {
    ROS_WARN("Odometry and IMU timestamps are not the same!");
    return;
  }

  // check if timestamps are 0
  if(ros::Time(0) == local_imu.header.stamp)
  {
    ROS_WARN("Didn't receive any new sensor message yet. Waiting...");
    return;
  }

  // set timestamp
  currentTimestamp = local_imu.header.stamp; // both imu and odo should be the same time


  // check if this is first loop or reinitialized
  if(ros::Time(0) == lastTimestamp){
    lastTimestamp = currentTimestamp;
    currentDelta = ros::Duration(0);
  }else{
    currentDelta = ( currentTimestamp - lastTimestamp );
  }


  // do all the kalman filter magic
  if(
       // 1. compute measurement update
       computeMeasurement(local_odo, local_imu) &&

       // 2. compute Kalman filter step
       computeFilterStep() &&

       // 3. publish car state
       publishCarState()
     )
  {

  }else{

    // something went wrong reinitialize Kalman Filter covariances
    initFilterCov();
  }

  // save timestamp
  lastTimestamp = currentTimestamp;

}

bool ImuOdoOdometry::computeMeasurement(const drive_ros_msgs::mav_cc16_ODOMETER_DELTA &odo_msg,
                                        const drive_ros_msgs::mav_cc16_IMU &imu_msg)
{
  // create measurement covariances
  Kalman::Covariance<Measurement> cov;
  cov.setZero();

  // time jump to big -> reset filter
  if(std::abs(currentDelta.toNSec()) > reset_filter_thres.toNSec()){

    ROS_ERROR_STREAM("Delta Time Threshold exceeded. Reset Filter."
        << " delta = " << currentDelta
        << " thres = " << reset_filter_thres);

    // time jumps bigger than 1 sec -> also reset Kalman state
    if(std::abs(currentDelta.sec) > 1)
    {
      initFilterState();
    }

    return false;

  // no sensor update :(
  } else if(currentDelta == ros::Duration(0)) {
      // Just predict using previous delta with low covariances
      ROS_WARN_STREAM("No new sensor data."
          << " last = " << lastTimestamp
          << " current = " << currentTimestamp);

      // increase no data counter
      ct_no_data++;

      // increase covariances because we have to use old sensor data
      cov(Measurement::AX,    Measurement::AX)    = imu_acc_cov_xx        * (ct_no_data+1);
      cov(Measurement::AY,    Measurement::AY)    = imu_acc_cov_yy        * (ct_no_data+1);
      cov(Measurement::V,     Measurement::V)     = odometer_velo_cov_xx  * (ct_no_data+1);
      cov(Measurement::OMEGA, Measurement::OMEGA) = imu_gyro_cov_zz       * (ct_no_data+1);


  // jumping back in time
  } else if( currentDelta < ros::Duration(0)) {
      ROS_WARN_STREAM("Jumping back in time."
          << " delta = " << currentDelta);

      // reset times
      currentTimestamp = lastTimestamp;
      currentDelta = ros::Duration(0);

      // increase no data counter
      ct_no_data++;

      // increase covariances because there is something fishy
      cov(Measurement::AX,    Measurement::AX)    = imu_acc_cov_xx        * (ct_no_data+1);
      cov(Measurement::AY,    Measurement::AY)    = imu_acc_cov_yy        * (ct_no_data+1);
      cov(Measurement::V,     Measurement::V)     = odometer_velo_cov_xx  * (ct_no_data+1);
      cov(Measurement::OMEGA, Measurement::OMEGA) = imu_gyro_cov_zz       * (ct_no_data+1);

  // everything is ok
  }else{

    // correct data available, reset counter
    ct_no_data = 0;

    // if current Delta is bigger than cycle time -> make error bigger
    // TODO: investigate a little bit more on this
    double factor_err = currentDelta.toSec() / rate.expectedCycleTime().toSec();
    ROS_DEBUG_STREAM("error factor: " << factor_err);

    cov(Measurement::AX,    Measurement::AX)    = imu_acc_cov_xx        * factor_err;
    cov(Measurement::AY,    Measurement::AY)    = imu_acc_cov_yy        * factor_err;
    cov(Measurement::V,     Measurement::V)     = odometer_velo_cov_xx  * factor_err;
    cov(Measurement::OMEGA, Measurement::OMEGA) = imu_gyro_cov_zz       * factor_err;

  }

  // Set measurement covariances
  mm.setCovariance(cov);

  // set measurements vector z
  z.v()     = odo_msg.velocity.x;
  z.ax()    = imu_msg.acc.x;
  z.ay()    = imu_msg.acc.y;
  z.omega() = imu_msg.gyro.z;

  ROS_DEBUG_STREAM("delta current: " << currentDelta << " delta previous: " << previousDelta);
  ROS_DEBUG_STREAM("measurementVector: " << z);
  ROS_DEBUG_STREAM("measurementCovariance:" << mm.getCovariance());

  if( std::isnan(cov(Measurement::AX,    Measurement::AX)   ) ||
      std::isnan(cov(Measurement::AY,    Measurement::AY)   ) ||
      std::isnan(cov(Measurement::V,     Measurement::V)    ) ||
      std::isnan(cov(Measurement::OMEGA, Measurement::OMEGA)) ||
      std::isnan(z.v()                                      ) ||
      std::isnan(z.ax()                                     ) ||
      std::isnan(z.ay()                                     ) ||
      std::isnan(z.omega()) )
  {
    ROS_ERROR("Measurement is NAN! Reinit covariances.");
    return false;
  }

  return true;
}

bool ImuOdoOdometry::computeFilterStep()
{
  // no new data avialable
  if(ros::Duration(0) == currentDelta) {
      // use time delta from previous step
      u.dt() = previousDelta.toSec();

  // new data available
  } else if(ros::Duration(0) != currentDelta) {

      // get current time delta
      u.dt() = currentDelta.toSec();

      // Update previous delta
      previousDelta = currentDelta;

  }

  // predict state for current time-step using the kalman filter
  filter.predict(sys, u);

  // perform measurement update
  filter.update(mm, z);

  return true;
}

bool ImuOdoOdometry::publishCarState()
{
  const auto& state = filter.getState();
  ROS_DEBUG_STREAM("stateCovariance" << filter.getCovariance());
  ROS_DEBUG_STREAM("newState: " << state);

  // check if nan
  if( std::isnan(state.x())       ||
      std::isnan(state.y())       ||
      std::isnan(state.theta())   ||
      std::isnan(state.v())       ||
      std::isnan(state.a())       ||
      std::isnan(state.omega())   ||
      std::isnan(z.omega()) )
  {
    ROS_ERROR("Measurement is NAN! Reinit Kalman.");
    initFilterState();
    return false;

  }


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
    marker.lifetime = ros::Duration(60);
    vis_pub.publish(marker);
  }

  return true;

}

