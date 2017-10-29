#ifndef IMUODOODOMETRY_H
#define IMUODOODOMETRY_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "drive_ros_msgs/mav_cc16_IMU.h"
#include "drive_ros_msgs/mav_cc16_ODOMETER_DELTA.h"
#include "drive_ros_msgs/TimeCompare.h"

#include <kalman/UnscentedKalmanFilter.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
#include "measurement_model.h"
#include "system_model.h"




class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& pnh);

  //! Destructor.
  ~ImuOdoOdometry();

  //! Initializer
  bool init();

  typedef float T;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;

  typedef message_filters::sync_policies::ApproximateTime<drive_ros_msgs::mav_cc16_ODOMETER_DELTA,
                                                          drive_ros_msgs::mav_cc16_IMU>
                                                          ApproxSyncPolicy;


private:

  void computeTimeStamp();
  void computeMeasurement();
  void computeFilterStep();
  void initFilter();
  void updateCarState();

  //! Callback function for subscriber.
  void syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                    const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu);

  message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU> *imu_sub;
  message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA> *odo_sub;
  message_filters::Synchronizer<ApproxSyncPolicy> *sync;
  ApproxSyncPolicy* policy;


  ros::Publisher time_debug_test;

  ros::NodeHandle pnh_;

  Control u;
  Measurement z;

  SystemModel sys;
  MeasurementModel mm;

  Filter filter;

  ros::Time lastTimestamp;
  ros::Time currentTimestamp;
  ros::Duration previousDelta;

};

#endif
