#ifndef IMUODOODOMETRY_H
#define IMUODOODOMETRY_H

#include <mutex>
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

#define GRAVITY T(9.81)


class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& pnh);

  //! Destructor.
  ~ImuOdoOdometry();

  void computeOdometry();

  typedef float T;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;


private:

  void computeMeasurement(const drive_ros_msgs::mav_cc16_ODOMETER_DELTA &odo_msg,
                          const drive_ros_msgs::mav_cc16_IMU &imu_msg);
  void computeFilterStep();
  void updateCarState();


  //! Callback function for subscriber.
  void odoCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg);
  void imuCallback(const drive_ros_msgs::mav_cc16_IMUConstPtr &msg);

  drive_ros_msgs::mav_cc16_ODOMETER_DELTA odo_msg;
  drive_ros_msgs::mav_cc16_IMU imu_msg;
  std::mutex odo_mut;
  std::mutex imu_mut;


  ros::Subscriber imu_sub;
  ros::Subscriber odo_sub;


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

  double max_time_diff;
  bool debug_time;

  double odometer_velo_cov_xx;
  double imu_acc_cov_xx;
  double imu_acc_cov_yy;
  double imu_gyro_cov_zz;
  double imu_acc_bias_x;
  double imu_acc_bias_y;
  double imu_acc_bias_z;
  double imu_gyro_bias_x;
  double imu_gyro_bias_y;
  double imu_gyro_bias_z;



};

#endif
