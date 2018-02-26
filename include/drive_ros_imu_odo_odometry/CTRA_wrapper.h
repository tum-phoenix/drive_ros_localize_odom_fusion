#ifndef CTRA_MODEL_H
#define CTRA_MODEL_H

#include "kalman/ExtendedKalmanFilter.hpp"
#include "base_wrapper.h"
#include "CTRA_measurement_model.h"
#include "CTRA_system_model.h"

class CTRAWrapper : public BaseWrapper
{
public:
  typedef float T;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;


  // constructor
  CTRAWrapper(ros::NodeHandle& n, ros::NodeHandle& p);


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
