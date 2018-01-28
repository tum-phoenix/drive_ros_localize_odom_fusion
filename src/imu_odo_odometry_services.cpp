#include "drive_ros_imu_odo_odometry/imu_odo_odometry.h"

// reload process covariances
bool ImuOdoOdometry::svr_reload_proc_cov(std_srvs::Trigger::Request  &req,
                                         std_srvs::Trigger::Response &res)
{
  initFilterProcessCov();

  res.message = "Kalman filter process covariances reloaded from parameter server.";
  return res.success = true;
}

// reinit kalman state
bool ImuOdoOdometry::svr_reinit_state(std_srvs::Trigger::Request  &req,
                                      std_srvs::Trigger::Response &res)
{
  initFilterState();

  res.message = "Kalman filter state reinitialized. Set state to 0 and load initial state covariances";
  return res.success = true;
}