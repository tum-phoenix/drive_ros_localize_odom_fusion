#include "drive_ros_imu_odo_odometry/CTRV_wrapper.h"


CTRVWrapper::CTRVWrapper(ros::NodeHandle& n, ros::NodeHandle& p)
{
  nh = n; pnh = p;

}



bool CTRVWrapper::initFilterState()
{
  ROS_INFO("Reset Kalman State.");

  // Init kalman
  State s;
  s.setZero();
  filter.init(s);

  // Set initial state covariance
  Kalman::Covariance<State> stateCov;
  stateCov.setZero();

  if(!(pnh.getParam("kalman_cov/filter_init_var_x", stateCov(State::X, State::X)) &&
       pnh.getParam("kalman_cov/filter_init_var_y", stateCov(State::Y, State::Y)) &&
       pnh.getParam("kalman_cov/filter_init_var_v", stateCov(State::V, State::V)) &&
       pnh.getParam("kalman_cov/filter_init_var_theta", stateCov(State::THETA, State::THETA)) &&
       pnh.getParam("kalman_cov/filter_init_var_omega", stateCov(State::OMEGA, State::OMEGA))))
  {
    ROS_ERROR("Error loading Kalman initial state covariance!");
    return false;
  }

  filter.setCovariance(stateCov);
  return true;

}

// initialize Filter covariances
bool CTRVWrapper::initFilterProcessCov()
{

  // Set process noise covariance
  Kalman::Covariance<State> cov;
  cov.setZero();

  if(!(pnh.getParam("kalman_cov/sys_var_x", cov(State::X, State::X)) &&
       pnh.getParam("kalman_cov/sys_var_y", cov(State::Y, State::Y)) &&
       pnh.getParam("kalman_cov/sys_var_v", cov(State::V, State::V)) &&
       pnh.getParam("kalman_cov/sys_var_theta", cov(State::THETA, State::THETA)) &&
       pnh.getParam("kalman_cov/sys_var_omega", cov(State::OMEGA, State::OMEGA))))
  {
    ROS_ERROR("Error loading Kalman process covariance!");
    return false;
  }

  sys.setCovariance(cov);
  return true;
}



bool CTRVWrapper::insertMeasurement(const nav_msgs::OdometryConstPtr &odo_msg,
                                     const sensor_msgs::ImuConstPtr &imu_msg)
{

  // Set measurement covariances
  Kalman::Covariance<Measurement> cov;
  cov.setZero();
  cov(Measurement::OMEGA, Measurement::OMEGA) = imu_msg->angular_velocity_covariance[CovElem::ang::angZ_angZ];
  cov(Measurement::V,     Measurement::V)     = odo_msg->twist.covariance[CovElem::lin_ang::linX_linX];
  mm.setCovariance(cov);


  // set measurements vector z
  z.v()     = odo_msg->twist.twist.linear.x;
  z.omega() = imu_msg->angular_velocity.z;

  ROS_DEBUG_STREAM("measurementVector: " << z);

  // check if there is something wrong
  if( std::isnan(cov(Measurement::V,     Measurement::V)    ) ||
      std::isnan(cov(Measurement::OMEGA, Measurement::OMEGA)) ||
      std::isnan(z.v()                                      ) ||
      std::isnan(z.omega()) )
  {
    ROS_ERROR("Measurement is NAN! Reinit Kalman.");
    // reset covariances and filter
    initFilterProcessCov();
    initFilterState();
    return false;
  }

  return true;
}


bool CTRVWrapper::computeFilterStep(const float delta,
                                    const nav_msgs::OdometryConstPtr &odo_msg,
                                    const sensor_msgs::ImuConstPtr &imu_msg)
{

  // time difference
  u.dt() = delta;

  // predict state for current time-step using the kalman filter
  filter.predict(sys, u);

  // perform measurement update
  filter.update(mm, z);

  return true;
}



bool CTRVWrapper::getOutput(geometry_msgs::TransformStamped& tf_msg,
                          nav_msgs::Odometry& odom_msg)
{
  // get new filter state
  const auto& state = filter.getState();
  ROS_DEBUG_STREAM("newState: " << state);

  // get new filter covariances
  const auto& cov_ft = filter.getCovariance();
  ROS_DEBUG_STREAM("FilterCovariance: " << cov_ft);

  // transform euler to quaternion angles
  tf2::Quaternion q1;
  q1.setRPY(0, 0, state.theta());

  // check if nan
  if( std::isnan(state.x())       ||
      std::isnan(state.y())       ||
      std::isnan(state.theta())   ||
      std::isnan(state.v())       ||
      std::isnan(state.omega()))
  {
    ROS_ERROR("State is NAN! Reinit Kalman.");
    // reset covariances and filter
    initFilterProcessCov();
    initFilterState();
    return false;

  }


  // tf
  tf_msg.transform.translation.x = state.x();
  tf_msg.transform.translation.y = state.y();
  tf_msg.transform.translation.z = 0;
  tf_msg.transform.rotation.x =  q1.x();
  tf_msg.transform.rotation.y =  q1.y();
  tf_msg.transform.rotation.z =  q1.z();
  tf_msg.transform.rotation.w =  q1.w();


  // odom pose
  odom_msg.pose.pose.position.x = state.x();
  odom_msg.pose.pose.position.y = state.y();
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.x = q1.x();
  odom_msg.pose.pose.orientation.y = q1.y();
  odom_msg.pose.pose.orientation.z = q1.z();
  odom_msg.pose.pose.orientation.w = q1.w();
  odom_msg.pose.covariance[CovElem::lin_ang::linX_linX] = cov_ft(State::X,      State::X);
  odom_msg.pose.covariance[CovElem::lin_ang::linX_linY] = cov_ft(State::X,      State::Y);
  odom_msg.pose.covariance[CovElem::lin_ang::linX_angZ] = cov_ft(State::X,      State::THETA);
  odom_msg.pose.covariance[CovElem::lin_ang::linY_linY] = cov_ft(State::Y,      State::Y);
  odom_msg.pose.covariance[CovElem::lin_ang::linY_linX] = cov_ft(State::Y,      State::X);
  odom_msg.pose.covariance[CovElem::lin_ang::linY_angZ] = cov_ft(State::Y,      State::THETA);
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_angZ] = cov_ft(State::THETA,  State::THETA);
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_linX] = cov_ft(State::THETA,  State::X);
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_linY] = cov_ft(State::THETA,  State::Y);

  // odom twist
  odom_msg.twist.twist.linear.x = state.v();
  odom_msg.twist.twist.angular.z = state.omega();
  odom_msg.twist.covariance[CovElem::lin_ang::linX_linX] = cov_ft(State::V,      State::V);
  odom_msg.twist.covariance[CovElem::lin_ang::linX_angZ] = cov_ft(State::V,      State::OMEGA);
  odom_msg.twist.covariance[CovElem::lin_ang::angZ_angZ] = cov_ft(State::OMEGA,  State::OMEGA);
  odom_msg.twist.covariance[CovElem::lin_ang::linX_angZ] = cov_ft(State::V,      State::OMEGA);


  return true;

}
