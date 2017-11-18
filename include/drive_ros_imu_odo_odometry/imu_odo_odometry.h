#ifndef IMUODOODOMETRY_H
#define IMUODOODOMETRY_H

// system
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>

// ros
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ros messages
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include "drive_ros_msgs/mav_cc16_IMU.h"
#include "drive_ros_msgs/mav_cc16_ODOMETER_DELTA.h"
#include "drive_ros_msgs/TimeCompare.h"

// kalman
#include <kalman/ExtendedKalmanFilter.hpp>
#include "measurement_model.h"
#include "system_model.h"


class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& pnh, ros::Rate& r);

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

  typedef message_filters::sync_policies::ExactTime<drive_ros_msgs::mav_cc16_ODOMETER_DELTA,
                                                    drive_ros_msgs::mav_cc16_IMU> SyncPolicy;


private:

  // initialize Kalman Filter
  void initFilterCov();
  void initFilterState();


  // collect data and prepare computing
  bool computeMeasurement(const drive_ros_msgs::mav_cc16_ODOMETER_DELTA &odo_msg,
                          const drive_ros_msgs::mav_cc16_IMU &imu_msg);

  // compute one kalman step
  bool computeFilterStep();

  // publish data
  bool publishCarState();


  // Callback function for subscriber
  void syncCallback(const drive_ros_msgs::mav_cc16_ODOMETER_DELTAConstPtr &msg_odo,
                    const drive_ros_msgs::mav_cc16_IMUConstPtr &msg_imu);



  // ROS subscriber + synchronizer
  message_filters::Subscriber<drive_ros_msgs::mav_cc16_IMU> *imu_sub;
  message_filters::Subscriber<drive_ros_msgs::mav_cc16_ODOMETER_DELTA> *odo_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;
  SyncPolicy* policy;

  // ROS publisher or broadcaster
  tf2_ros::TransformBroadcaster br;
  ros::Publisher vis_pub;

  ros::Rate rate;
  ros::NodeHandle pnh_;

  // ROS local message storage + mutex
  drive_ros_msgs::mav_cc16_ODOMETER_DELTA odo_msg;
  drive_ros_msgs::mav_cc16_IMU imu_msg;
  std::mutex mut;

  // kalman filter stuff
  Control u;
  Measurement z;
  SystemModel sys;
  MeasurementModel mm;
  Filter filter;

  // ROS times and durations
  ros::Time lastTimestamp;
  ros::Time currentTimestamp;
  ros::Duration currentDelta;

  // parameter
  ros::Duration reset_filter_thres;
  bool debug_rviz;
  bool debug_file;
  std::string tf_parent;
  std::string tf_child;

  // counter
  int ct_no_data;
  int ct;

  // debug to file
  std::ofstream file_log;

  // sensor covariance parameter
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
