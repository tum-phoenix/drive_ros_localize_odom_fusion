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
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "drive_ros_msgs/VehicleEncoder.h"
#include "drive_ros_msgs/TimeCompare.h"

// kalman
#include <kalman/ExtendedKalmanFilter.hpp>
#include "measurement_model.h"
#include "system_model.h"
#include "cov_elements.h"



class ImuOdoOdometry
{
public:
  //! Constructor.
  ImuOdoOdometry(ros::NodeHandle& nh, ros::NodeHandle& pnh, ros::Rate& r);

  //! Destructor.
  ~ImuOdoOdometry();

  void computeOdometry();

  typedef float T;

  typedef drive_ros_msgs::VehicleEncoder VeEnc;

  typedef CTRA::State<T> State;
  typedef CTRA::Control<T> Control;
  typedef CTRA::Measurement<T> Measurement;

  typedef CTRA::MeasurementModel<T> MeasurementModel;
  typedef CTRA::SystemModel<T> SystemModel;
  typedef Kalman::ExtendedKalmanFilter<State> Filter;

  typedef message_filters::sync_policies::ApproximateTime<drive_ros_msgs::VehicleEncoder,
                                                          sensor_msgs::Imu> SyncPolicy;


private:

  // initialize Kalman Filter
  void initFilterCov();
  void initFilterState();


  // collect data and prepare computing
  bool computeMeasurement(const drive_ros_msgs::VehicleEncoder &odo_msg,
                          const sensor_msgs::Imu &imu_msg);

  // compute one kalman step
  bool computeFilterStep();

  // publish data
  bool publishCarState();


  // Callback function for subscriber
  void syncCallback(const drive_ros_msgs::VehicleEncoderConstPtr &msg_odo,
                    const sensor_msgs::ImuConstPtr &msg_imu);



  // ROS subscriber + synchronizer
  message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
  message_filters::Subscriber<drive_ros_msgs::VehicleEncoder> *odo_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;
  SyncPolicy* policy;

  // ROS publisher or broadcaster
  tf2_ros::TransformBroadcaster br;
  ros::Publisher odo_pub;

  ros::Rate rate;
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // ROS local message storage + mutex
  drive_ros_msgs::VehicleEncoder odo_msg;
  sensor_msgs::Imu imu_msg;
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
  ros::Duration max_time_between_meas;
  bool debug_file;
  std::string static_frame;
  std::string moving_frame;

  // debug to file
  std::ofstream file_log;

};

#endif
