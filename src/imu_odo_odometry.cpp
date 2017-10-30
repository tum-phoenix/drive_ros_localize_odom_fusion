#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

ImuOdoOdometry::ImuOdoOdometry(ros::NodeHandle& pnh):
  pnh_(pnh)
{
  int queue_size;

  // ros parameters
  pnh.param<bool>("debug_time", debug_time, true);
  pnh.param<int>("queue_size", queue_size, 5);

  // init subscribers
  odo_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA>(pnh, "odo_in", queue_size);
  imu_sub = new message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU>(pnh, "imu_in", queue_size);

  policy = new SyncPolicy(queue_size);

  sync = new message_filters::Synchronizer<SyncPolicy>(static_cast<SyncPolicy>(*policy), *odo_sub, *imu_sub);
  sync->registerCallback(boost::bind(&ImuOdoOdometry::syncCallback, this, _1, _2));

  // reset initial times
  odo_msg.header.stamp = ros::Time(0);
  imu_msg.header.stamp = ros::Time(0);


  if(debug_time){
    time_debug_test = pnh.advertise<drive_ros_msgs::TimeCompare>("/odom/debug_times", 10);
  }

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

}



ImuOdoOdometry::~ImuOdoOdometry()
{

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
  drive_ros_msgs::mav_cc16_ODOMETER_DELTA local_odo;
  drive_ros_msgs::mav_cc16_IMU local_imu;
  ros::Duration diff;

  mut.lock();
  local_odo = odo_msg;
  local_imu = imu_msg;
  mut.unlock();

  diff = local_imu.header.stamp - local_odo.header.stamp;

  if(debug_time)
  {
    ROS_DEBUG_STREAM("diff time:" << diff.toNSec()*pow(10,-3));

    drive_ros_msgs::TimeCompare msg_time;
    msg_time.header.stamp = ros::Time::now();
    msg_time.time_1 = local_imu.header.stamp;
    msg_time.time_2 = local_odo.header.stamp;
    msg_time.diff_time = diff;
    time_debug_test.publish(msg_time);
  }

  if(ros::Time(0) == local_odo.header.stamp)
  {
    ROS_WARN("Didn't receive any odometry message yet. Waiting...");
    return;
  }


  if(ros::Time(0) == local_imu.header.stamp)
  {
    ROS_WARN("Didn't receive any IMU message yet. Waiting...");
    return;
  }

  // set timestamp
  currentTimestamp = local_imu.header.stamp; // both imu and odo should be the same time

  // Compute Measurement Update
  computeMeasurement(local_odo, local_imu);

  // Compute Kalman filter step
  computeFilterStep();

  // Update car state
  updateCarState();

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

void ImuOdoOdometry::computeFilterStep()
{
  // time since UKF was last called (parameter, masked as control input)
  auto currentDelta = ( currentTimestamp - lastTimestamp );
  if(currentDelta == ros::Duration(0)) {
      // Just predict using previous delta
      ROS_WARN_STREAM("No new sensor data"
          << " last = " << lastTimestamp
          << " current = " << currentTimestamp);

  } else if( currentDelta < ros::Duration(0)) {
      ROS_WARN_STREAM("Jumping backwards in time"
          << " last = " << lastTimestamp
          << " current = " << currentTimestamp
          << " delta = " << currentDelta);
      return;
  }

  ROS_DEBUG_STREAM("[delta]" << "delta current: " << currentDelta << " delta previous: " << previousDelta);

  if(ros::Duration(0) == currentDelta) {
      u.dt() = previousDelta.toSec();

      // Prediction only
      // predict state for current time-step using the kalman filter
      filter.predict(sys, u);
  } else {
      u.dt() = currentDelta.toSec();

      // Prediction + update
      // predict state for current time-step using the kalman filter
      filter.predict(sys, u);
      // perform measurement update
      filter.update(mm, z);

      // Update previous delta
      previousDelta = currentDelta;
  }

  const auto& state = filter.getState();

  ROS_DEBUG_STREAM("newState: " << state);
  ROS_DEBUG_STREAM("stateCovariance" <<filter.getCovariance());


}

void ImuOdoOdometry::updateCarState()
{

}

