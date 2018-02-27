#ifndef CTRV_WRAPPER_H
#define CTRV_WRAPPER_H

#include "kalman/ExtendedKalmanFilter.hpp"
#include "base_wrapper.h"
#include "CTRV_measurement_model.h"
#include "CTRV_system_model.h"

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
  bool initFilterProcessCov();

  bool insertMeasurement(const nav_msgs::OdometryConstPtr &odo_msg,
                         const sensor_msgs::ImuConstPtr &imu_msg);

  bool computeFilterStep(const float);

  bool getOutput(geometry_msgs::TransformStamped& tf_msg,
                 nav_msgs::Odometry& odom_msg);


  Control u;
  Measurement z;
  SystemModel sys;
  MeasurementModel mm;
  Filter filter;

};




#endif
