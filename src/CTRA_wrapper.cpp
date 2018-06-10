#include "drive_ros_localize_odom_fusion/CTRA_wrapper.h"


CTRAWrapper::CTRAWrapper(ros::NodeHandle& n, ros::NodeHandle& p)
{
  nh = n; pnh = p;
}



bool CTRAWrapper::initFilterState()
{
  bool ret = true;


  ROS_INFO("Reset Kalman State.");

  // Init kalman
  State s;
  s.setZero();
  filter.init(s);
  mm_cov.setZero();
  state_old.setZero();
  odom_old.setZero();
  yaw_old = 0;

  // Set initial state covariance
  Kalman::Covariance<State> stateCov;
  stateCov.setZero();

  ret &= pnh.getParam("kalman_cov/filter_init_var_x",     stateCov(State::X, State::X));
  ret &= pnh.getParam("kalman_cov/filter_init_var_y",     stateCov(State::Y, State::Y));
  ret &= pnh.getParam("kalman_cov/filter_init_var_theta", stateCov(State::THETA, State::THETA));
  ret &= filter.setCovariance(stateCov);

  // Set process noise covariance
  Kalman::Covariance<State> cov;
  cov.setZero();

  ret &= pnh.getParam("kalman_cov/sys_var_x",     cov(State::X, State::X));
  ret &= pnh.getParam("kalman_cov/sys_var_y",     cov(State::Y, State::Y));
  ret &= pnh.getParam("kalman_cov/sys_var_theta", cov(State::THETA, State::THETA));

  ret &= sys.setCovariance(cov);
  return ret;

}


bool CTRAWrapper::predict(const float delta,
                          const nav_msgs::OdometryConstPtr &odo_msg,
                          const sensor_msgs::ImuConstPtr &imu_msg)
{
  // check if the required messages are available
  if(imu_msg == NULL)
  {
    ROS_ERROR("Prediction imu message required for CTRA model! Abort.");
    return false;
  }

  // time difference
  u.dt()    = delta;
  u.omega() = imu_msg->angular_velocity.z;
  u.a() =     imu_msg->linear_acceleration.x;
  u.v() = std::sqrt(static_cast<float>(std::pow(odo_msg->twist.twist.linear.x, 2)
                                     + std::pow(odo_msg->twist.twist.linear.y, 2)));

  // predict state for current time-step using the kalman filter
  filter.predict(sys, u);

  // check if there is something wrong
  if(filter.getCovariance().hasNaN()            ||
     filter.getState().hasNaN()                 ||
     u.hasNaN() )
  {
    ROS_ERROR("State covariances or vector is broken! Abort!");
    return false;
  }

  return true;
}

bool CTRAWrapper::correct(const float delta,
                          const nav_msgs::OdometryConstPtr &odo_msg,
                          const sensor_msgs::ImuConstPtr &imu_msg)
{

  // check if the required messages are available
  if(odo_msg == NULL)
  {
    ROS_ERROR("Correction imu message required for CTRA model! Abort.");
    return false;
  }


  // Set measurement covariances
  const auto& cov_ft = filter.getCovariance();
  mm_cov(Measurement::X,    Measurement::X)     = odo_msg->pose.covariance[CovElem::lin_ang::linX_linX] - mm_cov(Measurement::X,    Measurement::X)  ;
  mm_cov(Measurement::X,    Measurement::Y)     = odo_msg->pose.covariance[CovElem::lin_ang::linX_linY] - mm_cov(Measurement::X,    Measurement::Y)  ;
  mm_cov(Measurement::X,    Measurement::YAW)   = odo_msg->pose.covariance[CovElem::lin_ang::linX_angZ] - mm_cov(Measurement::X,    Measurement::YAW);
  mm_cov(Measurement::Y,    Measurement::Y)     = odo_msg->pose.covariance[CovElem::lin_ang::linY_linY] - mm_cov(Measurement::Y,    Measurement::Y)  ;
  mm_cov(Measurement::Y,    Measurement::X)     = odo_msg->pose.covariance[CovElem::lin_ang::linY_linX] - mm_cov(Measurement::Y,    Measurement::X)  ;
  mm_cov(Measurement::Y,    Measurement::YAW)   = odo_msg->pose.covariance[CovElem::lin_ang::linY_angZ] - mm_cov(Measurement::Y,    Measurement::YAW);
  mm_cov(Measurement::YAW,  Measurement::X)     = odo_msg->pose.covariance[CovElem::lin_ang::angZ_linX] - mm_cov(Measurement::YAW,  Measurement::X)  ;
  mm_cov(Measurement::YAW,  Measurement::Y)     = odo_msg->pose.covariance[CovElem::lin_ang::angZ_linY] - mm_cov(Measurement::YAW,  Measurement::Y)  ;
  mm_cov(Measurement::YAW,  Measurement::YAW)   = odo_msg->pose.covariance[CovElem::lin_ang::angZ_angZ] - mm_cov(Measurement::YAW,  Measurement::YAW);
  mm.setCovariance(mm_cov);


  // get yaw
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(odo_msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // prevent yaw overflow
  if(yaw - yaw_old > M_PI){
    yaw -= 2*M_PI;
  }

  if(yaw_old - yaw > M_PI){
    yaw += 2*M_PI;
  }
  yaw_old = yaw;

  // set measurements vector z
  z.x()     = state_old.x()   + odo_msg->pose.pose.position.x - odom_old.x();
  z.y()     = state_old.y()   + odo_msg->pose.pose.position.y - odom_old.y();
  z.yaw()   = state_old.yaw() + yaw                           - odom_old.yaw();

  ROS_DEBUG_STREAM("measurementVector: " << z);

  // perform measurement update
  filter.update(mm, z);

  // check if there is something wrong
  if(mm_cov.hasNaN()            ||
     z.hasNaN())
  {
    ROS_ERROR("Measurement covariances or vector is broken! Abort.");
    return false;
  }

  state_old = filter.getState();
  odom_old.x() =   odo_msg->pose.pose.position.x;
  odom_old.y() =   odo_msg->pose.pose.position.y;
  odom_old.yaw() = yaw;

  return true;
}



bool CTRAWrapper::getOutput(geometry_msgs::TransformStamped& tf_msg,
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
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_linX] = cov_ft(State::THETA,  State::X);
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_linY] = cov_ft(State::THETA,  State::Y);
  odom_msg.pose.covariance[CovElem::lin_ang::angZ_angZ] = cov_ft(State::THETA,  State::THETA);

  return true;

}
