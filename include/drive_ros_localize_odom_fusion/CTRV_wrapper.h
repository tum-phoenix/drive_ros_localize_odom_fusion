#ifndef CTRV_WRAPPER_H
#define CTRV_WRAPPER_H

#include "kalman/ExtendedKalmanFilter.hpp"
#include "base_wrapper.h"
#include "CTRV_measurement_model.h"
#include "CTRV_system_model.h"
#include "drive_ros_localize_odom_fusion/moving_average.h"
#include <cmath>

// stupid clang compiler
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

class CTRVWrapper : public BaseWrapper
{
public:
  typedef float T;

  typedef CTRV::State<T> State;
  typedef CTRV::Control<T> Control;
  typedef CTRV::Measurement<T> Measurement;

  typedef CTRV::MeasurementModel<T> MeasurementModel;
  typedef CTRV::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;


  // constructor
  CTRVWrapper(ros::NodeHandle& n, ros::NodeHandle& p);


private:

  // initialize Kalman Filter
  bool initFilterState();

  bool predict(const float,
               const nav_msgs::OdometryConstPtr &odo_msg,
               const sensor_msgs::ImuConstPtr &imu_msg);

  bool correct(const float,
               const nav_msgs::OdometryConstPtr &odo_msg,
               const sensor_msgs::ImuConstPtr &imu_msg);

  bool getOutput(geometry_msgs::TransformStamped& tf_msg,
                 nav_msgs::Odometry& odom_msg);


  Control u;
  Measurement z;
  SystemModel sys;
  MeasurementModel mm;
  Filter filter;

  Measurement state_old;
  Measurement odom_old;
  double yaw_old;

};




#endif
