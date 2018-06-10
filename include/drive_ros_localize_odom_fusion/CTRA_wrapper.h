#ifndef CTRA_WRAPPER_H
#define CTRA_WRAPPER_H

#include "kalman/ExtendedKalmanFilter.hpp"
#include "base_wrapper.h"
#include "CTRA_measurement_model.h"
#include "CTRA_system_model.h"

// stupid clang compiler
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

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


  Kalman::Covariance<Measurement> mm_cov;
  Measurement odom_old;
  Measurement state_old;
  double yaw_old;
};




#endif
